"""Workstation-only two-UAV flight coordinator using existing AS2 clients/recovery."""
import json,math,signal,sys,time,threading
from pathlib import Path
import rclpy
from as2_msgs.action import Takeoff,Land
from as2_msgs.msg import PlatformStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from exploration_manager.msg import PairOpt,PairOptResponse
from bspline.msg import Bspline
from rosidl_runtime_py.convert import message_to_ordereddict
from base_mission import Mission as Base,xyz,ned_position
from completion import validate_budget,natural_finish,fleet_finished

FLEET=json.loads(Path('/config/fleet.json').read_text());CFG=json.loads(Path('/config/flight.json').read_text())
SURVEY=json.loads(Path('/evidence/survey/survey.json').read_text())
MODE=sys.argv[1];shared={};lock=threading.Lock();completion_record={}
EXPLORING=MODE in ('explore','explore-complete')
if MODE=='explore-complete':validate_budget(CFG.get('completion_wall_budget_s'))
ACTIVE=[v for v in FLEET['vehicles'] if MODE!='takeoff-drone1' or v['namespace']=='drone1']

class VehicleMission(Base):
    def __init__(self,v):
        self.v=v;ns=v['namespace'];self.ns=ns;self.native_state={};self.input_state={};self.triggered=False
        self.native_session=None;self.trigger_generation=0;self.max_exploration_displacement_m=0.
        root=Path('/result')/ns;root.mkdir()
        other=next(x for x in FLEET['vehicles'] if x['namespace']!=ns)
        super().__init__(root,False,ns,v['airsim'],v['px4_system_id'],other['namespace'],'/racer_multi/'+ns+'/adapter')
        self.destroy_publisher(self.target)
        self.target=self.create_publisher(PoseStamped,'/racer_multi/'+ns+'/trigger',10)
        self.create_subscription(String,'/racer_multi/'+ns+'/native_state',self.native,10)
        self.create_subscription(String,'/racer_multi/'+ns+'/input_state',self.inputs,10)
        self.pair_requests={};self.pair_responses={};self.shared_trajectories={}
        self.create_subscription(PairOpt,'/swarm_expl/pair_opt',self.pair_request,100)
        self.create_subscription(PairOptResponse,'/swarm_expl/pair_opt_res',self.pair_response,100)
        self.create_subscription(Bspline,'/planning/swarm_traj',self.swarm_traj,100)
        self.phase='preflight';self.create_timer(.1,self.share)
    def pair_request(self,m):
        key=f'{m.from_drone_id}:{m.to_drone_id}:{m.stamp}'
        if key not in self.pair_requests:self.pair_requests[key]=message_to_ordereddict(m);self.trace('pair_request',message=self.pair_requests[key])
    def pair_response(self,m):
        key=f'{m.to_drone_id}:{m.from_drone_id}:{m.stamp}'
        if key not in self.pair_responses:self.pair_responses[key]=message_to_ordereddict(m);self.trace('pair_response',message=self.pair_responses[key])
    def swarm_traj(self,m):
        self.shared_trajectories[m.drone_id]=m.traj_id
        if self.phase=='tracking':self.trace('shared_native_trajectory',message=message_to_ordereddict(m))
    def native(self,m):self.native_state=json.loads(m.data);self.rx['native']=time.monotonic()
    def inputs(self,m):
        self.input_state=json.loads(m.data);self.rx['inputs']=time.monotonic()
        self.trace('input_state',message=self.input_state)
    def share(self):
        natural=(self.triggered and self.fresh('native',1.25)
                 and natural_finish(self.native_state,self.native_session,self.trigger_generation))
        with lock:shared[self.ns]=dict(phase=self.phase,failed=getattr(self,'failed',False),
                                      natural_finished=natural,at=time.monotonic())
    def completion_ready(self):
        self.share()
        with lock:
            if completion_record:return True
        state=self.native_state
        if state.get('native_session')!=self.native_session:
            raise RuntimeError('native supervisor restarted during exploration')
        if state.get('fsm')=='FINISH' and not natural_finish(state,self.native_session,self.trigger_generation):
            raise RuntimeError('non-natural finish: '+str(state.get('completion_reason')))
        if state.get('completion_fault'):raise RuntimeError(state['completion_fault'])
        with lock:
            ready=fleet_finished(shared,[v['namespace'] for v in ACTIVE],time.monotonic(),1.25)
            if ready:completion_record.update(shared)
            return ready
    def wait_for_completion(self,started,result):
        budget=validate_budget(CFG.get('completion_wall_budget_s'))
        def done():
            if time.monotonic()-started>=budget:
                result['termination_reason']='budget_exhausted'
                raise RuntimeError('completion budget exhausted; exploration incomplete')
            if self.completion_ready():return True
            return False
        self.wait(done,max(.1,budget-(time.monotonic()-started))+1,'completion wait timed out')
        result['termination_reason']='native_finished'
        result['exploration_complete']=True
        with lock:result['fleet_completion']=dict(completion_record)

    def all_phase(self,phases):
        with lock:return len(shared)==len(ACTIVE) and all(x['phase'] in phases for x in shared.values())
    def check_motion(self):
        if self.recovery:return
        with lock:
            if any(x['failed'] for x in shared.values()):raise RuntimeError('peer mission failed; recover independently')
        if self.phase not in ['arming','takeoff','airborne','tracking']:return
        if not all(self.fresh(k,1.25) for k in ['platform','px4','odom','truth','peer']):raise RuntimeError('flight telemetry stale')
        if self.data['px4'].failsafe or not self.platform().connected:raise RuntimeError('platform failsafe/disconnected')
        truth=self.data['truth']
        if truth.get('error'):raise RuntimeError(truth['error'])
        for kind in ['rendered','physics']:
            p=truth['vehicles'][self.vehicle][kind]['position'];offset=[p[k+'_val']-self.air_start[i] for i,k in enumerate('xyz')]
            lo,hi=CFG['physical_height_during_active_flight_m']
            if math.hypot(*offset[:2])>CFG['physical_radius_m'] or not lo<=-offset[2]<=hi:raise RuntimeError('outside declared physical envelope')
        positions=[]
        for v in FLEET['vehicles']:
            p=truth['vehicles'][v['airsim']]['physics']['position'];positions.append([p[k+'_val']+v['spawn_ned'][i] for i,k in enumerate('xyz')])
        if math.dist(*positions)<CFG['peer_min_separation_m']:raise RuntimeError('minimum fleet separation violated')
        if self.phase=='tracking':
            p=truth['vehicles'][self.vehicle]['physics']['position']
            distance=math.dist([p[k+'_val'] for k in 'xy'],self.exploration_start_xy)
            self.max_exploration_displacement_m=max(self.max_exploration_displacement_m,distance)
            if not self.platform().armed or not self.platform().offboard:raise RuntimeError('lost armed/Offboard')
            if not all(self.fresh(k,1.25) for k in ['native','inputs']):raise RuntimeError('native/input state stale')
            now=self.get_clock().now().nanoseconds
            ages={k:(now-self.input_state.get(k,0))/1e9 for k in ['last_pair_ns','last_odom_ns']}
            limits={'last_pair_ns':CFG['depth_sample_max_age_s'],'last_odom_ns':CFG['odometry_sample_max_age_s']}
            if any(not 0<=age<=limits[k] for k,age in ages.items()):
                self.trace('input_freshness_failed',ages_s=ages,input_state=self.input_state)
                raise RuntimeError('real depth/odometry samples stale: '+str(ages))
            if self.native_state.get('process_failure') or self.adapter.stop_failure:raise RuntimeError(str(self.native_state.get('process_failure') or self.adapter.stop_failure))
    def recover_fleet_member(self,result):
        self.recovery=True;self.phase='stopping';self.share()
        if self.triggered:
            try:
                self.adapter.begin_stop(result.get('termination_reason','bounded fleet episode ended'),True)
                self.wait(lambda:not self.adapter.stopping or bool(self.adapter.stop_failure),6,'behavior stop timed out')
                if self.adapter.stop_failure:raise RuntimeError(self.adapter.stop_failure)
                self.wait(lambda:self.native_state.get('fsm')=='FINISH' and self.fresh('native',1.25),3,'native stop not confirmed')
                result['stop_confirmed']=True
            except Exception as exc:result['stop_error']=str(exc)
        if result['motion_requested']:
            try:
                if not self.disarmed():
                    self.phase='landing';self.share()
                    self.action(self.land,Land.Goal(land_speed=.35),'land_success',CFG['land_action_timeout_s'])
                    self.wait(lambda:self.disarmed() and self.fresh('land_detected',1.) and self.data['land_detected'].landed,10,'joint landed/disarmed timeout')
                    result['native_land_success']=True
            except Exception as exc:
                result['landing_error']=str(exc)
                try:self.px4_land();result['px4_recovery_landing']=True
                except Exception as failure:result['recovery_error']=str(failure)
        result['landed_disarmed']=self.disarmed() and self.fresh('land_detected',1.) and self.data['land_detected'].landed
        self.phase='finished';self.share()
    def run(self):
        result=dict(vehicle=self.ns,mode=MODE,motion_requested=False)
        if MODE=='explore-complete':result['exploration_complete']=False
        try:
            self.wait(lambda:all(self.fresh(k,1.25) for k in ['platform','px4','odom','peer','truth','land_detected','native','inputs'])
                      and self.adapter.ready() and self.takeoff.server_is_ready() and self.land.server_is_ready()
                      and self.native_state.get('fsm')=='WAIT_TRIGGER' and self.native_state.get('frontier_points',0)>0,90,'fleet capability readiness timeout')
            self.wait(lambda:self.disarmed() and self.data['land_detected'].landed and self.data['px4'].system_id==self.system_id
                      and self.data['px4'].pre_flight_checks_pass and not self.data['px4'].failsafe and not self.platform().offboard
                      and self.platform().status.state==PlatformStatus.DISARMED,25,'preflight unavailable')
            if not SURVEY[self.vehicle]['pass_takeoff']:raise RuntimeError('takeoff column blocked')
            self.air_start=ned_position(self.data['truth']);surveyed=SURVEY[self.vehicle]['pose_after']['position']
            if math.dist(self.air_start,[surveyed[k+'_val'] for k in 'xyz'])>.07:raise RuntimeError('moved since ground survey')
            self.anchor=self.position();self.phase='ready';self.share()
            self.wait(lambda:self.all_phase(['ready','arming','takeoff','airborne']),30,'peer preflight unavailable')
            self.phase='arming';self.share();result['motion_requested']=True
            self.service('set_arming_state',400,True);self.wait(lambda:self.platform().armed,5,'armed missing')
            self.service('set_offboard_mode',176,True);self.wait(lambda:self.platform().offboard,5,'Offboard missing')
            self.phase='takeoff';self.share()
            self.action(self.takeoff,Takeoff.Goal(takeoff_height=.75,takeoff_speed=.35),'takeoff_success',18)
            self.wait(lambda:self.platform().status.state==PlatformStatus.FLYING,5,'not FLYING')
            hold=time.monotonic()+1.2
            self.wait(lambda:time.monotonic()>=hold and math.hypot(*xyz(self.data['odom'].twist.twist.linear))<.18,5,'initial hover not settled')
            self.phase='airborne';self.share();result['takeoff_success']=True
            self.wait(lambda:self.all_phase(['airborne','tracking','stopping','landing','finished']),25,'peer takeoff incomplete')
            if EXPLORING:
                self.native_session=self.native_state.get('native_session')
                self.trigger_generation=self.native_state.get('trigger_generation',0)
                if MODE=='explore-complete' and not self.native_session:
                    raise RuntimeError('native lifecycle reporting unavailable')
                p=self.data['truth']['vehicles'][self.vehicle]['physics']['position']
                self.exploration_start_xy=[p[k+'_val'] for k in 'xy']
                result['exploration_start_xy']=self.exploration_start_xy
                self.phase='tracking';self.share();self.anchor=self.position();self.adapter.sync_context()
                msg=PoseStamped();msg.header.frame_id='racer_world';msg.header.stamp=self.get_clock().now().to_msg();msg.pose.orientation.w=1.
                self.target.publish(msg);self.triggered=True;started=time.monotonic()
                first_wait=min(CFG['first_active_wait_seconds'],CFG['completion_wall_budget_s']) if MODE=='explore-complete' else CFG['first_active_wait_seconds']
                self.wait(lambda:self.adapter.active>0,first_wait,'no native curve active')
                if MODE=='explore-complete':self.wait_for_completion(started,result)
                else:self.wait(lambda:time.monotonic()-started>=CFG['explore_wall_seconds'],CFG['explore_wall_seconds']+1,'episode timeout')
                active=self.adapter.snapshot.get('active_history',[]);p=self.data['truth']['vehicles'][self.vehicle]['physics']['position']
                movement=math.dist([p[k+'_val'] for k in 'xy'],self.air_start[:2])
                result.update(active_revisions=active,horizontal_displacement_m=movement,px4_references=self.px4_references,as2_references=self.as2_references)
                result['max_exploration_displacement_m']=self.max_exploration_displacement_m
                motion=self.max_exploration_displacement_m if MODE=='explore-complete' else movement
                if len(set(active))<CFG['minimum_active_revisions'] or motion<CFG['minimum_observed_displacement_m']:raise RuntimeError('insufficient native motion evidence')
                if not self.px4_references or not self.as2_references:raise RuntimeError('downstream consumption evidence missing')
        except BaseException as exc:
            if MODE=='explore-complete':
                timed_out='started' in locals() and time.monotonic()-started>=CFG['completion_wall_budget_s']
                result['termination_reason']='cancelled' if self.stop_requested else ('budget_exhausted' if timed_out else 'execution_failed')
                result['exploration_complete']=False
            self.failed=True;self.share();result['error']=str(exc);result['failed_phase']=self.phase
        finally:self.recover_fleet_member(result)
        result['cooperation']={'matched_accepted_pairs':{k:r for k,r in self.pair_responses.items() if k in self.pair_requests and r['status']==1},'native_pair_responses_consumed':self.native_state.get('accepted_pair_responses',0),'shared_trajectory_ids':self.shared_trajectories}
        result['pass']=bool(result.get('takeoff_success') and result['landed_disarmed'] and not any(k in result for k in ['error','stop_error','landing_error','recovery_error']) and (not EXPLORING or result.get('stop_confirmed')) and (MODE!='explore-complete' or result.get('exploration_complete')))
        (self.root/'result.json').write_text(json.dumps(result,indent=2));self.log.close();return result

if __name__=='__main__':
    assert MODE in ['takeoff','takeoff-drone1','explore','explore-complete'];rclpy.init(args=[])
    nodes=[VehicleMission(v) for v in ACTIVE];results={}
    def stop(*_):
        for n in nodes:n.stop_requested=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def worker(n):results[n.ns]=n.run()
    threads=[threading.Thread(target=worker,args=(n,)) for n in nodes]
    for t in threads:t.start()
    for t in threads:t.join()
    for n in nodes:n.e.shutdown();n.destroy_node()
    rclpy.try_shutdown();passed=all(r['pass'] for r in results.values()) and len(results)==len(nodes)
    if EXPLORING:
        pair_ok=any(r['cooperation']['matched_accepted_pairs'] and r['cooperation']['native_pair_responses_consumed']>0 for r in results.values())
        passed=passed and pair_ok
    else:pair_ok=None
    Path('/result/result.json').write_text(json.dumps({'mode':MODE,'pass':passed,'pairwise_consumption':pair_ok,'vehicles':results},indent=2))
    print(json.dumps(results,indent=2));raise SystemExit(0 if passed else 1)
