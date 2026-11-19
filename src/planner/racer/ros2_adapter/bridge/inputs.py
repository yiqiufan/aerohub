"""Use the validated camera adapter and adapt body velocity for native RACER."""
import copy
import signal
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


from camera_pose import CameraPoseAdapter, ODOM, BODY, quaternion, position


class Inputs(CameraPoseAdapter):
    def __init__(self, timing):
        super().__init__(timing)
        self.odom_out = self.create_publisher(Odometry, "/racer_short/odometry", 10)

    def on_platform(self, msg):
        # Sensing must continue while the aircraft is flying.
        self.platform_ready = msg.connected

    def on_odometry(self, msg):
        super().on_odometry(msg)
        if msg.header.frame_id != ODOM or msg.child_frame_id != BODY:
            return
        odom = copy.deepcopy(msg)
        rotation = Rotation.from_quat(quaternion(msg.pose.pose.orientation))
        linear = rotation.apply(position(msg.twist.twist.linear))
        angular = rotation.apply(position(msg.twist.twist.angular))
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear.tolist()
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular.tolist()
        odom.child_frame_id = ODOM
        self.odom_out.publish(odom)


if __name__ == "__main__":
    # Let the spin loop return before invalidating the ROS context. The default
    # signal handler can otherwise close it while spin_once builds its wait set.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def request_stop(*_):
        global stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    with open("/result/adapter-timing.jsonl", "w", buffering=1) as log:
        node = Inputs(log)
        try:
            while not stopping and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
