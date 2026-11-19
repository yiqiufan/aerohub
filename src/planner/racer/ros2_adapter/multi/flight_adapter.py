"""Per-UAV native RACER -> real AS2 execution; enabled only by workstation context."""
import json,math,signal,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
from exploration_manager.msg import TrajectoryExecution
from rosidl_runtime_py.convert import message_to_ordereddict
from online_adapter import OnlineAdapter
from stop_evidence import StopPolicy
from common import VEHICLE,NS,PREFIX,OUT,to_local_message

class FlightAdapter(OnlineAdapter):
    def __init__(self,log):
        self.log=log;self.registration=None;self.input_rx=0.;self.context_id=None;self.stop_id=None
        self.phase='preflight';self.active_history=[];self.accepted_plans={};self.operator_ended=False
        self.cfg=json.loads(Path('/config/flight.json').read_text())
        super().__init__(VEHICLE['racer_id'],NS,self.cfg['execution_max_speed_m_s'],self.record,
            StopPolicy(position_hold_contract='Fixed onboard v0.1.1 PID position-hold; existing single AirSim evidence'))
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.events=self.create_publisher(String,PREFIX+'/adapter/event',100)
        self.state_pub=self.create_publisher(String,PREFIX+'/adapter/state',10)
        self.create_subscription(String,PREFIX+'/input_state',self.inputs,10)
        self.create_subscription(String,PREFIX+'/adapter/command',self.command,10)
        self.create_timer(.04,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def record(self,event,**values):
        row=dict(event=event,wall_ns=time.time_ns(),sim_ns=self.now_ns(),vehicle=NS,**values)
        self.log.write(json.dumps(row)+'\n')
        if event=='active':self.active_history.append(values['plan'])
        if hasattr(self,'events'):self.events.publish(String(data=json.dumps(row)))
    def inputs(self,msg):
        self.registration=json.loads(msg.data);self.input_rx=time.monotonic()
    def command(self,msg):
        value=json.loads(msg.data)
        if value['command']=='context':
            if self.operator_ended:return
            self.phase=value['phase'];self.context_id=value['identity']
            self.record('context_accepted',context=value)
        elif value['command']=='stop' and self.stop_id!=value['identity']:
            self.begin_stop(value['detail'],operator=bool(value['operator']))
            self.stop_id=value['identity']
    def on_plan(self,msg):
        if self.disabled:return
        self.record('native_plan_received',message=message_to_ordereddict(msg))
        if self.phase!='tracking':
            self.report(msg.traj_id,TrajectoryExecution.REJECTED,'workstation has not enabled exploration');return
        if not self.registration or time.monotonic()-self.input_rx>1.25 or self.registration['offset'] is None:
            self.report(msg.traj_id,TrajectoryExecution.REJECTED,'common-frame registration unavailable or stale');return
        super().on_plan(to_local_message(msg,self.registration['offset']))
    def goal_for(self,plan):
        goal=super().goal_for(plan)
        offset=self.registration['offset']
        for t in plan.sample_times():
            p,v,a,y=plan.sample(t)
            if not all(math.isfinite(x) for x in (*p,*v,*a,y)):raise ValueError('non-finite curve')
            for axis,(lo,hi) in enumerate(self.cfg['execution_box_enu']):
                if not lo<=p[axis]+offset[axis]<=hi:raise ValueError('curve outside declared common-frame execution box')
            if math.hypot(*v)>self.speed:raise ValueError('curve exceeds existing execution speed bound')
        self.record('native_plan_validated',revision=plan.id,offset=offset,goal=message_to_ordereddict(goal))
        return goal
    def accept(self,pending):
        self.accepted_plans[pending.plan.id]=pending.plan
        super().accept(pending)
    def begin_stop(self,reason,operator=False):
        if self.operator_ended and not self.stopping:return
        if operator:self.operator_ended=True
        super().begin_stop(reason,operator)
    def report(self,identity,status,detail):
        self.record('trajectory_status',revision=identity,status=status,detail=detail)
        super().report(identity,status,detail)
    def publish(self):
        self.state_pub.publish(String(data=json.dumps(dict(namespace=NS,racer_id=self.vehicle,
            ready=self.ready(),dry_run=False,context_id=self.context_id,stop_id=self.stop_id,
            active=self.active,stopping=self.stopping,stop_failure=self.stop_failure,
            active_history=self.active_history,goal_handle=bool(self.goal_handle),pending=bool(self.pending),
            accepted_plans={str(k):dict(start=v.start) for k,v in self.accepted_plans.items()}))))

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with (OUT/'adapter-events.jsonl').open('x',buffering=1) as log:
        node=FlightAdapter(log)
        while running:rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
