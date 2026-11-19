"""Workstation native-exploration task: trigger only, never supplies a viewpoint."""
import json
import math
from pathlib import Path
import signal
import sys
import time
import rclpy
from as2_msgs.action import Takeoff
from as2_msgs.msg import PlatformStatus
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String
from base_mission import Mission as BaseMission, xyz, ned_position, heading

CFG=json.loads(Path('/config/config.json').read_text())

class Mission(BaseMission):
    def __init__(self,root,dry_run):
        super().__init__(root,dry_run)
        self.destroy_publisher(self.target)
        self.target=self.create_publisher(PoseStamped,'/racer_explore/trigger',10)
        self.create_subscription(String,'/racer_explore/native_state',self.native,10)
        self.native_state={}
        self.triggered=False
        self.land_completed_at=None
        self.survey=json.loads((root/'survey.json').read_text())
    def native(self,msg):
        value=json.loads(msg.data)
        if value!=self.native_state:
            self.trace('native_state',state=value)
        self.native_state=value;self.rx['native']=time.monotonic()
    def check_motion(self):
        if self.recovery or self.phase not in ('arming','takeoff','tracking'):return
        if not all(self.fresh(k,1.25) for k in ('px4','platform','odom','truth','peer')):
            raise RuntimeError('required flight telemetry stale')
        if self.data['px4'].failsafe or not self.platform().connected:raise RuntimeError('PX4 failsafe/disconnected')
        if self.data['peer'].arming_state!=1:raise RuntimeError('Drone1 unexpectedly armed')
        truth=self.data['truth']
        if truth.get('error'):raise RuntimeError('observer error: '+truth['error'])
        for source in ('rendered','physics'):
            p=truth['vehicles']['Drone0'][source]['position']
            n,e,d=(p[k+'_val']-self.air_start[i] for i,k in enumerate('xyz'))
            lo,hi=CFG['physical_height_during_active_flight_m']
            if math.hypot(n,e)>CFG['physical_radius_m'] or not lo<=-d<=hi:
                raise RuntimeError(f'{source} outside declared exploration envelope: {n,e,-d}')
        positions=[]
        for vehicle in ('Drone0','Drone1'):
            p=truth['vehicles'][vehicle]['physics']['position']
            spawn=self.survey['settings']['Vehicles'][vehicle]
            positions.append([p[k+'_val']+spawn.get(k.upper(),0) for k in 'xyz'])
        if math.dist(*positions)<CFG['peer_min_separation_m']:raise RuntimeError('approached parked Drone1')
        if self.phase=='tracking':
            if not self.platform().armed or not self.platform().offboard:raise RuntimeError('lost armed/Offboard')
            if not all(self.fresh(k,1.25) for k in ('map','camera','native')):raise RuntimeError('exploration input/supervisor stale')
            if self.native_state.get('process_failure'):raise RuntimeError(self.native_state['process_failure'])
            if self.adapter.stop_failure:raise RuntimeError(self.adapter.stop_failure)
    def trigger(self):
        self.adapter.sync_context()
        msg=PoseStamped()
        msg.header.frame_id='drone0/odom';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.orientation.w=1.
        self.target.publish(msg)
        self.triggered=True
        self.trace('native_exploration_trigger',semantic='FSM start only; pose ignored by original callback')
    def action(self,client,goal,result_field,timeout):
        super().action(client,goal,result_field,timeout)
        if result_field=='land_success':self.land_completed_at=time.monotonic()
    def recover(self,result):
        self.recovery=True;self.phase='stopping'
        if self.dry_run and self.triggered:
            try:
                self.adapter.begin_stop('ground shadow ended',operator=True)
                self.wait(lambda:self.native_state.get('fsm')=='FINISH' and self.disarmed()
                          and self.fresh('land_detected',1.) and self.data['land_detected'].landed
                          and self.fresh('peer',1.) and self.data['peer'].arming_state==1,
                          4,'native shadow stop and fresh ground confirmation missing')
                self.trace('ground_stop_confirmed',landed=True,disarmed=True,
                           land_feedback_age_s=time.monotonic()-self.rx['land_detected'])
                result['ground_native_stop_acknowledged']=True
            except Exception as error:result['ground_stop_error']=str(error)
        if not self.dry_run and result['motion_requested']:
            try:
                self.adapter.begin_stop('bounded native exploration ended',operator=True)
                self.wait(lambda:not self.adapter.stopping or bool(self.adapter.stop_failure),6,'stop protocol timed out')
                if self.adapter.stop_failure:raise RuntimeError(self.adapter.stop_failure)
                result['behavior_stop_confirmed']=True
                self.wait(lambda:self.fresh('native',1.25) and self.native_state.get('fsm')=='FINISH',3,
                          'native FSM did not acknowledge operator stop')
                result['native_stop_acknowledged']=True
            except Exception as error:result['native_stop_error']=str(error)
        super().recover(result)
        if not self.dry_run and result['motion_requested'] and self.land_completed_at is not None:
            # Use the original ten-second post-Land budget for BOTH asynchronous
            # confirmations; disarm can arrive before the land-detector update.
            try:
                remaining=max(0.,self.land_completed_at+10.-time.monotonic())
                self.wait(lambda:self.disarmed() and self.fresh('land_detected',1.)
                          and self.data['land_detected'].landed,remaining,
                          'landed/disarmed confirmation missing within original post-Land budget')
                result['disarmed']=True;result['landed']=True
                self.trace('final_ground_confirmed',landed=True,disarmed=True)
            except Exception as error:result['final_confirmation_error']=str(error)
    def run(self):
        result=dict(dry_run=self.dry_run,motion_requested=False,observer_collision_polling=False)
        try:
            self.wait(lambda:all(self.fresh(k,1.25) for k in ('platform','px4','odom','peer','truth','map','camera','native'))
                      and self.data['map'].width>0 and self.target.get_subscription_count()>0
                      and self.native_state.get('fsm')=='WAIT_TRIGGER' and self.native_state.get('frontier_points',0)>0
                      and self.adapter.ready() and self.takeoff.server_is_ready() and self.land.server_is_ready(),
                      35,'native FSM/frontiers/real perception/AS2 not ready')
            if not self.disarmed() or self.data['peer'].arming_state!=1:raise RuntimeError('both vehicles must be disarmed')
            if self.data['px4'].system_id!=135 or self.data['peer'].system_id!=136:raise RuntimeError('unexpected system IDs')
            self.wait(lambda:self.data['px4'].pre_flight_checks_pass and not self.data['px4'].failsafe
                      and self.fresh('land_detected',1.25) and self.data['land_detected'].landed
                      and not self.platform().offboard and self.platform().status.state==PlatformStatus.DISARMED,
                      25,'PX4/AS2 preflight not ready')
            if not self.survey['pass_takeoff']:raise RuntimeError('takeoff column blocked')
            self.air_start=ned_position(self.data['truth'])
            surveyed=ned_position(dict(pose=self.survey['vehicle_pose_after'][0]))
            if math.dist(self.air_start,surveyed)>.07:raise RuntimeError('aircraft moved since survey')
            self.anchor=self.position();self.yaw=heading(self.data['odom'].pose.pose.orientation)
            self.trace('preflight_ready',pos=self.anchor,airsim_ned=self.air_start,native=self.native_state)
            if self.dry_run:
                self.trigger()
                self.wait(lambda:self.adapter.candidate is not None,15,'no valid native exploration curve in ground shadow')
                result.update(ground_pass=True,ground_revision=self.adapter.candidate.id)
            else:
                self.phase='arming';result['motion_requested']=True
                self.service('set_arming_state',400,True)
                self.wait(lambda:self.platform().armed,5,'armed state missing')
                self.service('set_offboard_mode',176,True)
                self.wait(lambda:self.platform().offboard,5,'Offboard state missing')
                self.phase='takeoff'
                self.action(self.takeoff,Takeoff.Goal(takeoff_height=.75,takeoff_speed=.35),'takeoff_success',18)
                self.wait(lambda:self.platform().status.state==PlatformStatus.FLYING,5,'AS2 not FLYING')
                hold=time.monotonic()+1.2
                self.wait(lambda:time.monotonic()>=hold and math.hypot(*xyz(self.data['odom'].twist.twist.linear))<.18,5,'initial hover not settled')
                self.anchor=self.position();self.phase='tracking';self.trigger()
                started=time.monotonic()
                self.wait(lambda:self.adapter.active>0,CFG['first_active_wait_seconds'],'native exploration did not activate first curve')
                until=started+CFG['explore_wall_seconds']
                self.wait(lambda:time.monotonic()>=until or self.native_state.get('fsm')=='FINISH',
                          CFG['explore_wall_seconds']+1,'bounded exploration wait failed')
                active=self.adapter.snapshot.get('active_history',[])
                physics=self.data['truth']['vehicles']['Drone0']['physics']['position']
                movement=math.dist([physics[k+'_val'] for k in 'xy'],self.air_start[:2])
                if len(set(active))<CFG['minimum_active_revisions'] or movement<CFG['minimum_observed_displacement_m']:
                    raise RuntimeError('insufficient native replanning or physical movement evidence')
                result.update(active_revisions=active,view_selections=self.native_state['view_selections'],
                              horizontal_displacement_m=movement,px4_references=self.px4_references,
                              as2_references=self.as2_references,termination='bounded episode; not map completion')
        except BaseException as error:
            result.update(error=str(error),failed_phase=self.phase,native_state=self.native_state,
                          observed=sorted(self.data),fresh=sorted(k for k in self.data if self.fresh(k,1.25)))
            self.trace('failed',error=str(error))
        finally:self.recover(result)
        result['pass']=(result['disarmed'] and result['landed'] and result['peer_disarmed'] and 'error' not in result and 'ground_stop_error' not in result and 'final_confirmation_error' not in result
                        and (result.get('ground_pass',False) if self.dry_run else
                             result.get('behavior_stop_confirmed',False) and result.get('native_stop_acknowledged',False)
                             and 'landing_error' not in result and 'native_stop_error' not in result))
        (self.root/('ground-result.json' if self.dry_run else 'flight-result.json')).write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        return 0 if result['pass'] else 1

if __name__=='__main__':
    rclpy.init(args=[])
    mission=Mission(Path('/result'),'--fly' not in sys.argv)
    signal.signal(signal.SIGINT,lambda *_:setattr(mission,'stop_requested',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(mission,'stop_requested',True))
    code=mission.run();mission.e.shutdown();mission.destroy_node();rclpy.try_shutdown()
    raise SystemExit(code)
