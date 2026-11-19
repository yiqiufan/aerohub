"""Opt-in, per-vehicle rosbag2 recording. No commands or AirSim RPC calls."""
import json
import queue
import resource
import sqlite3
from pathlib import Path
from contextlib import closing
import signal
import threading
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from common import NS, PREFIX, CONFIG, OUT, VEHICLE


def topics():
    sensor = '/' + NS + '/sensor_measurements/front_center/depth_planar'
    return {
        sensor + '/image': 'sensor_msgs/msg/Image',
        sensor + '/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/' + NS + '/navigation/odom': 'nav_msgs/msg/Odometry',
        '/' + NS + '/platform/info': 'as2_msgs/msg/PlatformInfo',
        '/' + NS + '/motion_reference/trajectory': 'as2_msgs/msg/TrajectorySetpoints',
        '/' + NS + '/fmu/out/vehicle_status_v4': 'px4_msgs/msg/VehicleStatus',
        '/' + NS + '/fmu/out/vehicle_land_detected': 'px4_msgs/msg/VehicleLandDetected',
        PREFIX + '/depth': 'sensor_msgs/msg/Image',
        PREFIX + '/camera_pose': 'geometry_msgs/msg/PoseStamped',
        PREFIX + '/odometry': 'nav_msgs/msg/Odometry',
        PREFIX + '/occupancy': 'sensor_msgs/msg/PointCloud2',
        PREFIX + '/occupancy_local': 'sensor_msgs/msg/PointCloud2',
        PREFIX + '/occupancy_local_inflate': 'sensor_msgs/msg/PointCloud2',
        PREFIX + '/input_state': 'std_msgs/msg/String',
        PREFIX + '/native_state': 'std_msgs/msg/String',
        PREFIX + '/shadow_state': 'std_msgs/msg/String',
        PREFIX + '/legacy_bspline': 'bspline/msg/Bspline',
        '/racer_execution/drone_' + str(VEHICLE['racer_id']) + '/plan': 'bspline/msg/Bspline',
        '/racer_execution/drone_' + str(VEHICLE['racer_id']) + '/status': 'exploration_manager/msg/TrajectoryExecution',
        PREFIX + '/adapter/event': 'std_msgs/msg/String',
        PREFIX + '/adapter/state': 'std_msgs/msg/String',
        '/clock': 'rosgraph_msgs/msg/Clock',
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }


def persisted_counts(bag):
    """Count finalized SQLite messages; does not claim payload decode or DDS completeness."""
    paths = sorted(Path(bag).glob('*.db3'))
    if not paths:
        raise RuntimeError('no finalized SQLite bag files')
    counts = Counter()
    for path in paths:
        with closing(sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)) as db:
            db.execute('PRAGMA query_only=ON')
            for topic, count in db.execute(
                    'SELECT t.name, COUNT(*) FROM messages m '
                    'JOIN topics t ON m.topic_id=t.id GROUP BY t.name'):
                counts[topic] += count
    return counts


class Recorder(Node):
    def __init__(self):
        super().__init__('racer_diagnostic_recorder_' + NS,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        config = json.loads((CONFIG.parent / 'diagnostics.json').read_text())
        if not config.get('enabled'):
            raise ValueError('diagnostics must be explicitly enabled')
        self.topics = topics()
        self.cache_bytes = int(config.get('bag_cache_bytes', 32 * 1024 * 1024))
        if self.cache_bytes < 0:raise ValueError('bag_cache_bytes must be nonnegative')
        self.pending = queue.Queue(maxsize=int(config.get('queue_messages', 128)))
        self.counts = Counter(); self.written = Counter(); self.dropped = Counter()
        self.error = ''; self.backlog = 0; self.max_callback_ns = 0
        self.ending = threading.Event(); self.ready = threading.Event()
        self.started = time.time_ns(); self.cpu_started = time.process_time()
        self.thread = threading.Thread(target=self.write, name='bag-writer')
        self.thread.start(); self.ready.wait(10)
        if not self.ready.is_set() or self.error:
            raise RuntimeError('writer failed to initialize: ' + self.error)
        self.subscriptions_kept = []
        for topic, type_name in self.topics.items():
            qos = qos_profile_sensor_data
            if topic == '/tf_static':
                qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            def make_callback(name):
                def callback(msg):
                    self.receive(name, msg)
                return callback
            self.subscriptions_kept.append(self.create_subscription(get_message(type_name), topic, make_callback(topic), qos))
        self.graph = (OUT / 'publishers.jsonl').open('x', buffering=1)
        self.create_timer(2., self.snapshot)
        (OUT / 'ready.json').write_text(json.dumps(dict(vehicle=NS,wall_ns=time.time_ns(),
            topics=self.topics,scope='writer open and subscriptions constructed; DDS delivery assessed separately')))

    def receive(self, topic, msg):
        begin = time.monotonic_ns(); wall = time.time_ns()
        self.counts[topic] += 1
        header = getattr(msg, 'header', None)
        stamp = header.stamp.sec * 10**9 + header.stamp.nanosec if header else None
        row = dict(topic=topic, sequence=self.counts[topic], wall_ns=wall,
                   sim_ns=self.get_clock().now().nanoseconds, header_stamp_ns=stamp,
                   publisher_identity='periodic graph only; locked rclpy callback has no MessageInfo')
        try:
            self.pending.put_nowait((topic, serialize_message(msg), wall, row))
            self.backlog = max(self.backlog, self.pending.qsize())
        except queue.Full:
            self.dropped[topic] += 1
        self.max_callback_ns = max(self.max_callback_ns, time.monotonic_ns() - begin)

    def write(self):
        writer = None
        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(OUT / 'bag'), storage_id='sqlite3',max_cache_size=self.cache_bytes),
                        rosbag2_py.ConverterOptions('', ''))
            for name, type_name in self.topics.items():
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr'))
            self.ready.set()
            with (OUT / 'receipts.jsonl').open('x') as receipts:
                while not self.ending.is_set() or not self.pending.empty():
                    try:
                        name, data, stamp, row = self.pending.get(timeout=.1)
                    except queue.Empty:
                        continue
                    writer.write(name, data, stamp)
                    receipts.write(json.dumps(row) + '\n')
                    self.written[name] += 1
        except BaseException as exc:
            self.error = repr(exc)
        finally:
            # The fixed Humble image exposes close(); finish cache flush before counting.
            if writer is not None:
                try:
                    writer.close()
                except BaseException as exc:
                    self.error = self.error or 'writer close failed: ' + repr(exc)
            del writer
            self.ready.set()

    def snapshot(self):
        endpoints = {name: [dict(node=x.node_name, namespace=x.node_namespace,
                                 gid=bytes(x.endpoint_gid).hex(), type=x.topic_type)
                            for x in self.get_publishers_info_by_topic(name)] for name in self.topics}
        self.graph.write(json.dumps(dict(wall_ns=time.time_ns(), topics=endpoints)) + '\n')

    def close(self):
        self.snapshot(); self.graph.close(); self.ending.set(); self.thread.join(10)
        persisted = Counter(); persistence_checked = False
        if self.thread.is_alive():
            self.error = self.error or 'writer failed to drain within 10 seconds'
        else:
            try:
                persisted = persisted_counts(OUT / 'bag')
                persistence_checked = True
                if persisted != self.written:
                    self.error = self.error or 'persisted bag counts differ from writer API submissions'
            except Exception as exc:
                self.error = self.error or 'persisted bag check failed: ' + repr(exc)
        result = dict(vehicle=NS, start_wall_ns=self.started, end_wall_ns=time.time_ns(),
                      received=dict(self.counts), written=dict(self.written), dropped=dict(self.dropped),
                      persisted=dict(persisted), persistence_checked=persistence_checked,
                      missing_from_bag=dict(self.written - persisted),
                      unexpected_in_bag=dict(persisted - self.written),
                      written_semantics='writer API submissions; persisted counts checked after close',
                      error=self.error, max_queue_messages=self.backlog, max_callback_ns=self.max_callback_ns,
                      process_cpu_seconds=time.process_time() - self.cpu_started,
                      max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                      bag_cache_bytes=self.cache_bytes,
                      bag_timestamp='wall reception; replay recorded /clock without --clock',
                      transport_limit='BEST_EFFORT observation; queue drops measured, DDS delivery loss not directly measured',
                      pass_=not self.error and not self.dropped and persistence_checked and self.counts == self.written == persisted)
        result['pass'] = result.pop('pass_')
        (OUT / 'recorder-result.json').write_text(json.dumps(result, indent=2))
        return result['pass']


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Recorder(); running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        while running and not node.error:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        okay = node.close(); node.destroy_node(); rclpy.try_shutdown()
    return 0 if okay else 1


if __name__ == '__main__':
    raise SystemExit(main())
