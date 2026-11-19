"""Actual per-vehicle sensor stream in a registered shared ENU map frame."""
import copy
import json
import signal
import time
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from camera_pose import CameraPoseAdapter, position, quaternion
from common import FLEET, VEHICLE, NS, PREFIX, CONFIG, OUT, translation

class Inputs(CameraPoseAdapter):
    def __init__(self, log):
        super().__init__(log,NS,PREFIX,str(CONFIG/'mapper.yaml'))
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.target_frame=FLEET['common_frame']
        self.offset=None;self.anchor=None;self.count=0;self.pairs=0
        self.last_odom_ns=0;self.last_pair_ns=0;self.registration_ground=False
        self.odom_out=self.create_publisher(Odometry,PREFIX+'/odometry',10)
        self.state_out=self.create_publisher(String,PREFIX+'/input_state',10)
        self.create_timer(.2,self.publish_state)

    def on_platform(self,msg):
        self.platform_ready=msg.connected
        self.registration_ground=msg.connected and not msg.armed and not msg.offboard

    def on_odometry(self,msg):
        super().on_odometry(msg)
        if msg.header.frame_id!=self.odom_frame or msg.child_frame_id!=self.body_frame:return
        if self.offset is None:
            if not self.registration_ground or not self.odometry:return
            self.anchor=position(msg.pose.pose.position).tolist()
            self.offset=translation(self.anchor,VEHICLE['world_ground_enu'])
            (OUT/'registration.json').write_text(json.dumps({'namespace':NS,'racer_id':VEHICLE['racer_id'],
                'local_ground_anchor':self.anchor,'world_from_odom_translation':self.offset,
                'source':'stationary AS2 odometry + configured surveyed spawn; aligned ENU axes',
                'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec},indent=2))
        out=copy.deepcopy(msg);p=self.map_position(position(msg.pose.pose.position))
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=p.tolist()
        rot=Rotation.from_quat(quaternion(msg.pose.pose.orientation))
        for name in ('linear','angular'):
            value=rot.apply(position(getattr(msg.twist.twist,name)))
            target=getattr(out.twist.twist,name);target.x,target.y,target.z=value.tolist()
        out.header.frame_id=out.child_frame_id=self.target_frame
        self.odom_out.publish(out);self.count+=1
        self.last_odom_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def map_position(self,p):return p+np.array(self.offset)
    def drain(self):
        if self.offset is not None:super().drain()
    def record(self,stamp,status,**values):
        super().record(stamp,status,**values)
        if status=='accepted':self.pairs+=1;self.last_pair_ns=stamp
    def publish_state(self):
        self.state_out.publish(String(data=json.dumps({'namespace':NS,'racer_id':VEHICLE['racer_id'],
            'offset':self.offset,'anchor':self.anchor,'odometry_count':self.count,'image_pairs':self.pairs,
            'last_odom_ns':self.last_odom_ns,'last_pair_ns':self.last_pair_ns,'frame':self.target_frame})))

if __name__=='__main__':
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    running=True
    def stop(*_):
        global running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with (OUT/'input-timing.jsonl').open('x',buffering=1) as log:
        node=Inputs(log)
        while running:rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.try_shutdown()
