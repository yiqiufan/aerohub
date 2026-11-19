"""Replay neither state nor depth. Pair live AS2 odometry with each AirSim image."""
from bisect import bisect_left
from collections import deque
import json
import math
from pathlib import Path
import time

from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


PREFIX = "/drone0/sensor_measurements/front_center/depth_planar"
ODOM = "drone0/odom"
BODY = "drone0/base_link"
OPTICAL = "drone0/front_center_optical"


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def position(value):
    return np.array([value.x, value.y, value.z], dtype=float)


def quaternion(value):
    return np.array([value.x, value.y, value.z, value.w], dtype=float)


class CameraPoseAdapter(Node):
    def __init__(self, timing, namespace="drone0", topic_prefix="/racer_ground", mapping_path="/config/mapper.yaml"):
        super().__init__("racer_ground_camera_pose_adapter" + ("" if namespace == "drone0" else "_" + namespace))
        self.timing = timing
        self.odom_frame = namespace + "/odom"
        self.body_frame = namespace + "/base_link"
        self.optical_frame = namespace + "/front_center_optical"
        self.target_frame = self.odom_frame
        prefix = "/" + namespace + "/sensor_measurements/front_center/depth_planar"
        self.odometry = deque(maxlen=200)
        self.images = {}
        self.camera_infos = {}
        self.last_stamp = 0
        self.last_clock = 0
        self.platform_ready = False
        params = yaml.safe_load(Path(mapping_path).read_text())["/**"]["ros__parameters"]["map_ros"]
        self.intrinsics = [params[key] for key in ("fx", "fy", "cx", "cy")]
        self.tf_buffer = Buffer(cache_time=Duration(seconds=3), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_out = self.create_publisher(Image, topic_prefix + "/depth", QoSProfile(depth=2))
        self.pose_out = self.create_publisher(PoseStamped, topic_prefix + "/camera_pose", QoSProfile(depth=10))
        self.create_subscription(Odometry, "/" + namespace + "/navigation/odom", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, "/" + namespace + "/platform/info", self.on_platform, qos_profile_sensor_data)
        self.create_subscription(Image, prefix + "/image", self.on_image, QoSProfile(depth=2))
        self.create_subscription(CameraInfo, prefix + "/camera_info", self.on_info, QoSProfile(depth=6))
        self.create_timer(0.01, self.drain)

    def map_position(self, point):
        return point

    def on_odometry(self, msg):
        stamp = stamp_ns(msg)
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.body_frame or stamp <= 0:
            return
        p, q = position(msg.pose.pose.position), quaternion(msg.pose.pose.orientation)
        if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.01:
            return
        if not self.odometry or stamp > self.odometry[-1][0]:
            self.odometry.append((stamp, p, q))

    def on_platform(self, msg):
        self.platform_ready = msg.connected and not msg.armed and not msg.offboard

    def on_image(self, msg):
        stamp = stamp_ns(msg)
        if stamp > self.last_stamp:
            self.images[stamp] = (time.monotonic(), msg)
            if len(self.images) > 10:
                del self.images[min(self.images)]

    def on_info(self, msg):
        stamp = stamp_ns(msg)
        if stamp > self.last_stamp:
            self.camera_infos[stamp] = msg
            if len(self.camera_infos) > 10:
                del self.camera_infos[min(self.camera_infos)]

    def record(self, stamp, status, **values):
        self.timing.write(json.dumps(dict(wall_ns=time.time_ns(), stamp_ns=stamp,
                                          status=status, **values)) + "\n")

    def match(self, stamp):
        times = [item[0] for item in self.odometry]
        index = bisect_left(times, stamp)
        if index < len(times) and times[index] == stamp:
            _, p, q = self.odometry[index]
            return p, Rotation.from_quat(q), 0.0, 0.0
        if not 0 < index < len(times):
            return None
        lo, p0, q0 = self.odometry[index - 1]
        hi, p1, q1 = self.odometry[index]
        if hi - lo > 60_000_000:
            return None
        ratio = (stamp - lo) / (hi - lo)
        rotation = Slerp([0, 1], Rotation.from_quat([q0, q1]))([ratio])[0]
        return (1 - ratio) * p0 + ratio * p1, rotation, (stamp - lo) / 1e6, (hi - stamp) / 1e6

    def publish_pair(self, stamp, image, camera, match, extrinsic, clock, queued):
        valid_layout = (
            image.encoding == "32FC1" and image.header.frame_id == camera.header.frame_id == self.optical_frame
            and (image.width, image.height) == (camera.width, camera.height) == (640, 480)
            and not image.is_bigendian and image.step == image.width * 4
            and len(image.data) == image.height * image.step
            and np.allclose([camera.k[i] for i in (0, 4, 2, 5)], self.intrinsics, atol=1e-6)
            and all(value == 0 for value in camera.d)
        )
        if not valid_layout:
            self.record(stamp, "camera_contract")
            return
        depth = np.frombuffer(image.data, dtype="<f4")
        if not np.isfinite(depth).all() or not (depth > 0).all():
            self.record(stamp, "invalid_depth")
            return
        body_pos, body_rot, before_ms, after_ms = match
        camera_pos = self.map_position(body_pos + body_rot.apply(position(extrinsic.translation)))
        camera_rot = body_rot * Rotation.from_quat(quaternion(extrinsic.rotation))
        msg = PoseStamped()
        msg.header.stamp = image.header.stamp
        msg.header.frame_id = self.target_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = camera_pos.tolist()
        (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
         msg.pose.orientation.w) = camera_rot.as_quat().tolist()
        self.pose_out.publish(msg)
        self.depth_out.publish(image)
        self.record(stamp, "accepted", before_ms=before_ms, after_ms=after_ms,
                    camera_position=camera_pos.tolist(), camera_quaternion=camera_rot.as_quat().tolist(),
                    body_position=body_pos.tolist(), body_quaternion=body_rot.as_quat().tolist(),
                    image_age_s=(clock - stamp) / 1e9, queue_wall_s=time.monotonic() - queued)

    def drain(self):
        clock = self.get_clock().now().nanoseconds
        if not clock:
            return
        if clock < self.last_clock:
            self.odometry.clear()
            self.images.clear()
            self.camera_infos.clear()
            self.last_stamp = 0
            self.record(clock, "clock_reset")
        self.last_clock = clock
        for stamp in sorted(self.images):
            queued, image = self.images[stamp]
            camera = self.camera_infos.get(stamp)
            match = self.match(stamp)
            if clock < stamp or not (camera and match and self.platform_ready):
                if time.monotonic() - queued < 0.6:
                    continue
                self.record(stamp, "pair_timeout", camera_info=bool(camera), odom_pair=match is not None,
                            platform_ready=self.platform_ready)
            elif clock - stamp > 500_000_000:
                self.record(stamp, "stale_image", age_s=(clock - stamp) / 1e9)
            else:
                try:
                    transform = self.tf_buffer.lookup_transform(self.body_frame, self.optical_frame, Time()).transform
                except TransformException:
                    if time.monotonic() - queued < 0.6:
                        continue
                    self.record(stamp, "mount_tf_missing")
                else:
                    self.publish_pair(stamp, image, camera, match, transform, clock, queued)
            self.last_stamp = max(self.last_stamp, stamp)
            del self.images[stamp]
            for old in [key for key in self.camera_infos if key <= self.last_stamp]:
                del self.camera_infos[old]


def main():
    rclpy.init()
    with open("/result/adapter-timing.jsonl", "w", buffering=1) as timing:
        node = CameraPoseAdapter(timing)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
