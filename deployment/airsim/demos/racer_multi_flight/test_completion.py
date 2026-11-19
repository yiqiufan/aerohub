"""No ROS, Docker or vehicles: run actual task callbacks against isolated state inputs."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import Mock
from completion import natural_finish, fleet_finished, validate_budget

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
spec=importlib.util.spec_from_file_location('native_lifecycle',ROOT/'src/tool/navigation/planner/racer/ros2_adapter/multi/native_lifecycle.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
NativeLifecycle=module.NativeLifecycle

def task_class():
    # Compile the real class body, excluding module-level ROS/config/flight entry effects.
    tree=ast.parse((HERE/'mission.py').read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VehicleMission')
    clock=types.SimpleNamespace(monotonic=lambda:10.)
    env=dict(Base=object,MODE='explore-complete',EXPLORING=True,json=json,math=math,
             time=clock,shared={},completion_record={},lock=threading.Lock(),
             ACTIVE=[{'namespace':'drone0'},{'namespace':'drone1'}],
             CFG={'completion_wall_budget_s':30},natural_finish=natural_finish,
             fleet_finished=fleet_finished,validate_budget=validate_budget)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(HERE/'mission.py'),'exec'),env)
    return env['VehicleMission'],env

def feed(lifecycle, line):
    result=lifecycle.feed(line)
    if ' to ' in line:result['fsm']=line.rsplit(' to ',1)[1]
    return result

def completed(lifecycle):
    feed(lifecycle,'[triggerCallback]: Drone 1 from WAIT_TRIGGER to PLAN_TRAJ')
    feed(lifecycle,'handoff stop requested: return finished')
    feed(lifecycle,'handoff status applied: status=3 id=9 queue_ms=0')
    return feed(lifecycle,'[handoff stopped]: Drone 1 from EXEC_TRAJ to FINISH')

class CompletionTests(unittest.TestCase):
    def setUp(self):self.cls,self.env=task_class()
    def task(self,name='drone0'):
        obj=self.cls.__new__(self.cls)
        obj.ns=name;obj.native_state={};obj.rx={};obj.phase='tracking';obj.failed=False
        obj.triggered=True;obj.native_session=None;obj.trigger_generation=0
        obj.fresh=lambda key,age:self.env['time'].monotonic()-obj.rx.get(key,0)<=age
        return obj
    def update(self,obj,state):
        obj.native(types.SimpleNamespace(data=json.dumps(state)))
    def test_idle_is_not_completion_and_reallocation_can_resume(self):
        life=NativeLifecycle(1);feed(life,'Drone 1 from WAIT_TRIGGER to PLAN_TRAJ')
        feed(life,'handoff stop requested: no grid');feed(life,'handoff status applied: status=3')
        for end in ('IDLE','PLAN_TRAJ'):
            state=feed(life,'Drone 1 from EXEC_TRAJ to '+end)
            self.assertFalse(natural_finish(state,life.session,0))
    def test_return_requires_stop_feedback(self):
        life=NativeLifecycle(1);feed(life,'Drone 1 from WAIT_TRIGGER to PLAN_TRAJ')
        feed(life,'handoff stop requested: return finished')
        self.assertFalse(natural_finish(feed(life,'Drone 1 from EXEC_TRAJ to FINISH'),life.session,0))
    def test_operator_stop_overrides_return_request(self):
        life=NativeLifecycle(1);feed(life,'Drone 1 from WAIT_TRIGGER to PLAN_TRAJ')
        feed(life,'handoff stop requested: return finished');feed(life,'handoff status applied: status=4')
        state=feed(life,'Drone 1 from EXEC_TRAJ to FINISH')
        self.assertEqual(state['completion_reason'],'operator_stopped')
        self.assertFalse(natural_finish(state,life.session,0))
    def test_initial_empty_and_old_generation_are_not_completion(self):
        life=NativeLifecycle(1);state=feed(life,'Drone 1 from WAIT_TRIGGER to FINISH')
        self.assertEqual(state['completion_reason'],'initial_no_frontier')
        self.assertFalse(natural_finish(state,life.session,0))
        life=NativeLifecycle(1);state=completed(life)
        self.assertTrue(natural_finish(state,life.session,0))
        self.assertFalse(natural_finish(state,life.session,1))
    def test_solver_failure_not_disguised_as_empty_work(self):
        life=NativeLifecycle(1);state=completed(life)
        state.update(life.feed('Fail to solve ATSP.'))
        self.assertFalse(natural_finish(state,life.session,0))
    def test_actual_native_callback_and_two_member_barrier(self):
        a,b=self.task(),self.task('drone1')
        for obj in (a,b):
            life=NativeLifecycle(1);obj.native_session=life.session;self.update(obj,completed(life))
        self.assertFalse(a.completion_ready()) # one machine cannot finish the fleet
        b.share();self.assertTrue(a.completion_ready())
        # The first thread's normal cleanup may publish operator FINISH before peer wakes.
        a.native_state['completion_reason']='operator_stopped';a.share()
        self.assertTrue(b.completion_ready())
        self.assertEqual(set(self.env['completion_record']),{'drone0','drone1'})
    def test_stale_peer_and_restart_rejected(self):
        obj=self.task();life=NativeLifecycle(1);obj.native_session=life.session;self.update(obj,completed(life))
        self.env['shared']['drone1']=dict(natural_finished=True,failed=False,at=0)
        self.assertFalse(obj.completion_ready())
        obj.native_state['native_session']='restarted'
        with self.assertRaisesRegex(RuntimeError,'restarted'):obj.completion_ready()
    def test_actual_wait_budget_is_incomplete(self):
        obj=self.task();obj.completion_ready=lambda:False
        obj.wait=lambda condition,*args:condition()
        result={}
        with self.assertRaisesRegex(RuntimeError,'budget exhausted'):obj.wait_for_completion(-30,result)
        self.assertEqual(result['termination_reason'],'budget_exhausted')
        self.assertNotIn('exploration_complete',result)
    def test_actual_wait_natural_completion(self):
        obj=self.task();obj.completion_ready=lambda:True
        obj.wait=lambda condition,*args:condition()
        result={};obj.wait_for_completion(5,result)
        self.assertTrue(result['exploration_complete']);self.assertEqual(result['termination_reason'],'native_finished')
    def test_actual_recovery_preserves_reason_and_requires_stop(self):
        obj=self.task();obj.share=lambda:None
        obj.adapter=types.SimpleNamespace(begin_stop=Mock(),stopping=False,stop_failure=None)
        obj.native_state={'fsm':'FINISH'};obj.fresh=lambda *a:True
        obj.wait=lambda predicate,*a:self.assertTrue(predicate())
        obj.disarmed=lambda:True;obj.data={'land_detected':types.SimpleNamespace(landed=True)}
        result={'motion_requested':False,'termination_reason':'budget_exhausted','exploration_complete':False}
        obj.recover_fleet_member(result)
        obj.adapter.begin_stop.assert_called_once_with('budget_exhausted',True)
        self.assertTrue(result['stop_confirmed']);self.assertFalse(result['exploration_complete'])
    def test_invalid_budget_and_fleet_failure(self):
        for value in (None,0,-1,True,float('nan'),float('inf'),'100'):
            with self.assertRaises(ValueError):validate_budget(value)
        self.assertEqual(validate_budget(30),30)
        self.assertFalse(fleet_finished({'drone0':dict(natural_finished=True,failed=True,at=10)},['drone0'],10,1.25))
    def test_expired_budget_wins_over_late_finish(self):
        obj=self.task();obj.completion_ready=lambda:True;obj.wait=lambda condition,*args:condition()
        result={}
        with self.assertRaisesRegex(RuntimeError,'budget exhausted'):obj.wait_for_completion(-30,result)
        self.assertEqual(result['termination_reason'],'budget_exhausted')
    def test_invalid_complete_mode_fails_before_services(self):
        from full_flow import execute
        deployment=Mock();deployment.validate_completion_config.side_effect=ValueError('budget required')
        with self.assertRaises(ValueError):execute(deployment,'explore-complete')
        deployment.up.assert_not_called()
    def test_return_to_origin_still_has_real_motion_evidence(self):
        from evaluate import observed_excursion
        events=[dict(event='airsim',source={'vehicles':{'Drone0':{'physics':{'position':dict(x_val=x,y_val=0)}}}}) for x in (0,2,0)]
        self.assertEqual(observed_excursion(events,'Drone0',{'exploration_start_xy':[0,0]}),2)
        self.assertEqual(observed_excursion([events[0],events[2]],'Drone0',{'exploration_start_xy':[0,0]}),0)

if __name__=='__main__':unittest.main()
