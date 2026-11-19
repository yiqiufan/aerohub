"""Bound the native planner output without replacing autonomous target selection."""
import json
import math
from pathlib import Path
from exploration_manager.msg import TrajectoryExecution
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from online_adapter import OnlineAdapter
from stop_evidence import StopPolicy
from trajectory import Plan

CONFIG=json.loads(Path('/config/config.json').read_text())
class GuardedAdapter(OnlineAdapter):
    def __init__(self,mission,dry_run):
        super().__init__(1,'drone0',CONFIG['execution_max_speed_m_s'],mission.trace,
            StopPolicy(position_hold_contract='Fixed onboard v0.1.1 PID position-hold implementation; prior ground and AirSim flight evidence'))
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.mission,self.dry_run=mission,dry_run
        self.accepted_plans={};self.candidate=None
    def on_plan(self,msg):
        if self.disabled:return
        self.trace('native_plan_received',revision=msg.traj_id,message=message_to_ordereddict(msg))
        if self.dry_run:
            try:
                plan=Plan.parse(dict(vehicle=1,received_ns=self.now_ns(),message=message_to_ordereddict(msg)))
                self.goal_for(plan);self.candidate=plan
                self.trace('dry_run_validated',revision=plan.id)
            except (ValueError,RuntimeError,OverflowError) as e:self.trace('dry_run_rejected',error=str(e))
            self.report(msg.traj_id,TrajectoryExecution.REJECTED,'ground shadow; no execution authorized')
            return
        if self.mission.phase!='tracking':
            self.report(msg.traj_id,TrajectoryExecution.REJECTED,'task has not enabled exploration')
            return
        super().on_plan(msg)
    def goal_for(self,plan):
        goal=super().goal_for(plan)
        for t in plan.sample_times():
            p,v,a,y=plan.sample(t)
            if not all(math.isfinite(k) for k in (*p,*v,*a,y)):raise ValueError('non-finite curve')
            for i,(lo,hi) in enumerate(CONFIG['execution_box_enu']):
                if self.dry_run and i==2:lo=-.15
                if not lo<=p[i]<=hi:raise ValueError(f'curve outside bounded exploration box: {p}')
            if math.hypot(*v)>CONFIG['execution_max_speed_m_s']:raise ValueError('curve exceeds existing speed bound')
        self.trace('native_plan_validated',revision=plan.id,first=plan.sample(0)[0],end=plan.sample(plan.position.duration)[0],duration_s=plan.position.duration)
        return goal
    def accept(self,pending):
        self.accepted_plans[pending.plan.id]=pending.plan
        super().accept(pending)
    def report(self,identity,status,detail):
        self.trace('trajectory_status',revision=identity,status=status,detail=detail)
        super().report(identity,status,detail)
