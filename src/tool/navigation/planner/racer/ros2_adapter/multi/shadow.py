"""Ground-only request construction; never sends an Action goal or modification."""
import json
import math
import signal
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty,String
from exploration_manager.msg import TrajectoryExecution
from rosidl_runtime_py.convert import message_to_ordereddict
from online_adapter import OnlineAdapter
from trajectory import Plan
from common import FLEET,VEHICLE,NS,PREFIX,OUT,to_local_message

class Shadow(OnlineAdapter):
    def __init__(self,log):
        self.log=log;self.registration=None;self.validated=[];self.rejected=[];self.stopped=False
        super().__init__(VEHICLE['racer_id'],NS,1.5,self.record)
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.pub=self.create_publisher(String,PREFIX+'/shadow_state',10)
        self.create_subscription(String,PREFIX+'/input_state',self.inputs,10)
        self.create_subscription(Empty,PREFIX+'/operator_stop',self.on_operator_stop,10)
        self.create_timer(.1,self.publish)

    def record(self,event,**values):
        self.log.write(json.dumps({'event':event,'wall_ns':time.time_ns(),'sim_ns':self.now_ns(),**values})+'\n')
    def report(self, identity, status, detail):
        self.record('execution_status_sent',revision=identity,status=status,detail=detail)
        super().report(identity,status,detail)
    def inputs(self,msg):self.registration=json.loads(msg.data)
    def on_plan(self,msg):
        if self.disabled:return
        self.record('native_plan',message=message_to_ordereddict(msg))
        try:
            if msg.drone_id!=self.vehicle:raise ValueError('wrong native vehicle ID')
            if not self.ready():raise ValueError('AS2 execution endpoints unavailable')
            if not self.registration or self.registration['offset'] is None:raise ValueError('registration missing')
            offset=self.registration['offset']
            local=to_local_message(msg,offset)
            plan=Plan.parse({'vehicle':self.vehicle,'received_ns':self.now_ns(),'message':message_to_ordereddict(local)})
            self.session='ground-shadow-'+NS
            goal=super().goal_for(plan)
            for t in plan.sample_times():
                p,v,a,y=plan.sample(t)
                if not all(math.isfinite(x) for x in (*p,*v,*a,y)):raise ValueError('non-finite trajectory')
                if math.hypot(*v)>self.speed:raise ValueError('existing execution speed limit exceeded')
            assert goal.path.vehicle_id==NS and goal.path.frame_id==NS+'/odom'
            self.validated.append(msg.traj_id)
            self.record('shadow_validated',revision=msg.traj_id,offset=offset,goal=message_to_ordereddict(goal))
        except (ValueError,RuntimeError,OverflowError,AssertionError) as exc:
            self.rejected.append(str(exc));self.record('shadow_invalid',revision=msg.traj_id,error=str(exc))
        # Never acknowledge physical execution that did not occur.
        self.report(msg.traj_id,TrajectoryExecution.REJECTED,'ground-only shadow; no execution authorized')
        if msg.traj_id in self.validated:
            # One actual candidate is sufficient for routing inspection. Avoid an
            # unbounded rejection/replan loop while the fleet stays grounded.
            self.on_operator_stop(Empty())
    def on_stop(self,_):
        self.report(0,TrajectoryExecution.OPERATOR_STOPPED if self.stopped else TrajectoryExecution.STOPPED,
                    'ground-only shadow ended' if self.stopped else 'ground-only shadow; no physical goal was sent')
        self.record('shadow_stop_ack')
    def on_operator_stop(self,_):
        self.disabled=True;self.stopped=True
        self.report(0,TrajectoryExecution.OPERATOR_STOPPED,'ground-only operator stop; no motion requested')
    def publish(self):
        self.pub.publish(String(data=json.dumps({'namespace':NS,'racer_id':self.vehicle,'ready':self.ready(),
            'validated':self.validated,'invalid':self.rejected,'stopped':self.stopped,'motion_requested':False})))

if __name__=='__main__':
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    running=True
    def stop(*_):
        global running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with (OUT/'shadow-events.jsonl').open('x',buffering=1) as log:
        node=Shadow(log)
        while running:rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.try_shutdown()
