"""Onboard adapter, local odometry and AS2 Action clients; task control over ROS only."""
import json
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace


import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from exploration_manager.msg import TrajectoryExecution
from guard import GuardedAdapter
from remote_adapter import PREFIX


def main():
    rclpy.init(args=[])
    log = (Path('/result') / 'adapter-events.jsonl').open('x', buffering=1)
    ctx = SimpleNamespace(phase='preflight', anchor=None, goal=None, yaw=0.0)
    state = dict(context_id=None, stop_id=None, odom=None, odom_rx=0.0, active_history=[])
    adapter = None
    def trace(event, **values):
        row = dict(event=event, wall_ns=time.time_ns(), sim_ns=adapter.now_ns() if adapter else 0, **values)
        log.write(json.dumps(row) + '\n')
        if event == 'active':
            state['active_history'].append(values['plan'])
        if adapter:
            events.publish(String(data=json.dumps(row)))
    def position():
        if state['odom'] is None or time.monotonic() - state['odom_rx'] > 1.25:
            raise ValueError('onboard odometry unavailable or stale')
        p = state['odom'].pose.pose.position
        return (p.x, p.y, p.z)
    ctx.position, ctx.trace = position, trace
    dry_run = '--dry-run' in sys.argv
    adapter = GuardedAdapter(ctx, dry_run)
    events = adapter.create_publisher(String, PREFIX + '/event', 100)
    status = adapter.create_publisher(String, PREFIX + '/state', 10)
    def odometry(msg):
        state['odom'], state['odom_rx'] = msg, time.monotonic()
    adapter.create_subscription(Odometry, '/drone0/navigation/odom', odometry, qos_profile_sensor_data)
    def command(msg):
        value = json.loads(msg.data)
        if value['command'] == 'context':
            ctx.phase, ctx.anchor, ctx.goal, ctx.yaw = (value[k] for k in ('phase', 'anchor', 'goal', 'yaw'))
            state['context_id'] = value['identity']
            trace('context_accepted', context=value)
        elif value['command'] == 'stop':
            if state['stop_id'] != value['identity']:
                if dry_run:
                    adapter.disabled = True
                    adapter.report(0, TrajectoryExecution.OPERATOR_STOPPED,
                                   'ground shadow ended; no physical execution was authorized')
                    trace('dry_run_stopped')
                else:
                    adapter.begin_stop(value['detail'], operator=value['operator'])
                state['stop_id'] = value['identity']
    adapter.create_subscription(String, PREFIX + '/command', command, 10)
    def publish():
        status.publish(String(data=json.dumps(dict(
            ready=adapter.ready(), dry_run=dry_run, context_id=state['context_id'], stop_id=state['stop_id'],
            active=adapter.active, stopping=adapter.stopping, stop_failure=adapter.stop_failure,
            active_history=state['active_history'],
            goal_handle=bool(adapter.goal_handle), pending=bool(adapter.pending),
            candidate=adapter.candidate.id if adapter.candidate else None,
            accepted_plans={str(k): dict(start=v.start) for k, v in adapter.accepted_plans.items()}))))
    adapter.create_timer(.04, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    while running:
        rclpy.spin_once(adapter, timeout_sec=.05)
    adapter.destroy_node()
    rclpy.try_shutdown()
    log.close()


if __name__ == '__main__':
    main()
