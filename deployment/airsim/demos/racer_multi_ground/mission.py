"""Workstation ground probe: trigger native FSMs, observe cooperation, stop; no flight API."""
from collections import defaultdict
import json,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,String
from px4_msgs.msg import VehicleStatus,VehicleLandDetected
from as2_msgs.msg import PlatformInfo
from exploration_manager.msg import DroneState,PairOpt,PairOptResponse
from plan_env.msg import ChunkStamps,ChunkData
from bspline.msg import Bspline
from rosidl_runtime_py.convert import message_to_ordereddict

FLEET=json.loads(Path('/config/fleet.json').read_text());OUT=Path('/result')
class Probe(Node):
    def __init__(self):
        super().__init__('racer_multi_ground_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.rows={v['namespace']:{} for v in FLEET['vehicles']};self.rx={};self.logs=(OUT/'events.jsonl').open('x',buffering=1)
        self.commands={};self.states={};self.stamps={};self.trajs={};self.opt={};self.responses={};self.chunks=defaultdict(int)
        self.arming=defaultdict(set);self.position_samples={};self.abort=False;self.phase='readiness'
        for v in FLEET['vehicles']:
            ns=v['namespace'];prefix='/racer_multi/'+ns
            for key,typ,topic in [('inputs',String,prefix+'/input_state'),('native',String,prefix+'/native_state'),
                                 ('shadow',String,prefix+'/shadow_state'),('odom',Odometry,prefix+'/odometry'),
                                 ('platform',PlatformInfo,'/'+ns+'/platform/info'),
                                 ('px4',VehicleStatus,'/'+ns+'/fmu/out/vehicle_status_v4'),
                                 ('land',VehicleLandDetected,'/'+ns+'/fmu/out/vehicle_land_detected')]:
                qos=10 if typ==String else qos_profile_sensor_data
                self.create_subscription(typ,topic,lambda m,n=ns,k=key:self.on_vehicle(n,k,m),qos)
            self.commands[ns]=(self.create_publisher(PoseStamped,prefix+'/trigger',10),
                               self.create_publisher(Empty,prefix+'/operator_stop',10))
        for typ,topic,cb in [(DroneState,'/swarm_expl/drone_state',self.on_state),
                             (PairOpt,'/swarm_expl/pair_opt',self.on_opt),
                             (PairOptResponse,'/swarm_expl/pair_opt_res',self.on_response),
                             (ChunkStamps,'/multi_map_manager/chunk_stamps',self.on_stamp),
                             (ChunkData,'/multi_map_manager/chunk_data',self.on_chunk),
                             (Bspline,'/planning/swarm_traj',self.on_traj)]:self.create_subscription(typ,topic,cb,100)
    def record(self,event,**kw):self.logs.write(json.dumps({'event':event,'phase':self.phase,'wall_ns':time.time_ns(),**kw})+'\n')
    def on_vehicle(self,ns,key,msg):
        value=json.loads(msg.data) if isinstance(msg,String) else message_to_ordereddict(msg)
        self.rows[ns][key]=value;self.rx[ns,key]=time.monotonic()
        if key=='px4':
            self.arming[ns].add(msg.arming_state)
            if msg.arming_state!=1:self.abort=True
        if key=='odom':self.position_samples[ns]=value
        if key in ('native','shadow','inputs') or key in ('land','px4'):self.record(key,vehicle=ns,value=value)
    def on_state(self,m):self.states[m.drone_id]=message_to_ordereddict(m);self.record('drone_state',value=self.states[m.drone_id])
    def on_stamp(self,m):
        self.stamps[m.from_drone_id]=[list(x.ids) for x in m.idx_lists]
        self.record('chunk_stamps',sender=m.from_drone_id,idx_lists=self.stamps[m.from_drone_id])
    def on_chunk(self,m):
        self.chunks[f'{m.from_drone_id}->{m.to_drone_id}:owner{m.chunk_drone_id}']+=1
        self.record('chunk_data',sender=m.from_drone_id,receiver=m.to_drone_id,owner=m.chunk_drone_id,index=m.idx,voxel_count=len(m.voxel_adrs))
    def on_opt(self,m):
        key=f'{m.from_drone_id}:{m.to_drone_id}:{m.stamp}'
        if key not in self.opt:self.opt[key]=message_to_ordereddict(m);self.record('pair_opt',value=self.opt[key])
    def on_response(self,m):
        key=f'{m.to_drone_id}:{m.from_drone_id}:{m.stamp}'
        if key not in self.responses:self.responses[key]=message_to_ordereddict(m);self.record('pair_response',value=self.responses[key])
    def on_traj(self,m):self.trajs[m.drone_id]=m.traj_id
    def fresh(self,ns,key,age=2.):return time.monotonic()-self.rx.get((ns,key),0)<age
    def wait(self,test,seconds):
        end=time.monotonic()+seconds
        while not test():
            if self.abort:raise RuntimeError('ground abort or armed state observed')
            if time.monotonic()>end:raise TimeoutError('timeout in '+self.phase)
            rclpy.spin_once(self,timeout_sec=.03)
    def ready(self):
        for v in FLEET['vehicles']:
            ns=v['namespace'];d=self.rows[ns]
            if not all(self.fresh(ns,k) for k in ['inputs','native','shadow','platform','px4','land','odom']):return False
            if not (d['px4']['arming_state']==1 and d['land']['landed'] and d['platform']['connected'] and not d['platform']['armed']):return False
            if not (d['inputs']['image_pairs']>0 and d['inputs']['odometry_count']>0 and d['inputs']['offset'] is not None):return False
            if not (d['native']['fsm']=='WAIT_TRIGGER' and d['native']['frontier_points']>0 and d['shadow']['ready']):return False
            if d['native']['process_failure']:raise RuntimeError(ns+': '+d['native']['process_failure'])
        return True
    def run(self):
        result={'scope':str(len(FLEET['vehicles']))+'-UAV PC ground shadow, no motion or execution acceptance','motion_requested':False}
        try:
            self.wait(self.ready,FLEET['readiness_timeout_s'])
            self.record('ready',rows=self.rows,world_odometry=self.position_samples)
            result['readiness']=True
            self.phase='native_cooperation'
            for ns,(trigger,_) in self.commands.items():
                m=PoseStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id=FLEET['common_frame'];m.pose.orientation.w=1.
                trigger.publish(m);self.record('trigger',vehicle=ns)
            end=time.monotonic()+FLEET['observation_seconds'];self.wait(lambda:time.monotonic()>=end,FLEET['observation_seconds']+1)
            ids={v['racer_id'] for v in FLEET['vehicles']}
            paired={key:r for key,r in self.responses.items() if key in self.opt and r['status']==1}
            peers={r['from_drone_id'] for r in paired.values()}|{r['to_drone_id'] for r in paired.values()}
            chunks={str(i):[j for j in sorted(ids-{i}) if len(self.stamps.get(i,[]))==FLEET['native_slots'] and self.stamps[i][j-1]] for i in sorted(ids)}
            checks={'all_native_candidates':all(d.get('shadow',{}).get('validated') for d in self.rows.values()),
                    'all_native_states':ids<=set(self.states),'all_stationary_swarm_trajectories':ids<=set(self.trajs),
                    'pairwise_exchange_reaches_all_vehicles':ids<=peers,
                    'native_initiator_consumed_response':any(d['native'].get('accepted_pair_responses',0)>0 for d in self.rows.values()),
                    'each_map_integrates_foreign_chunks':all(chunks[str(i)] for i in ids),
                    'all_input_frames':all(d['odom']['header']['frame_id']==FLEET['common_frame'] for d in self.rows.values()),
                    'all_shadow_targets_ready':all(d['shadow']['ready'] for d in self.rows.values()),
                    'no_arming':all(self.arming[v['namespace']]=={1} for v in FLEET['vehicles'])}
            result.update(checks=checks,chunk_owners_by_receiver=chunks,pair_requests=len(self.opt),matched_accepted_pairs=paired,
                          observed_drone_states=self.states,shared_trajectory_ids=self.trajs,chunk_data_counts=dict(self.chunks))
            topics=['/swarm_expl/drone_state','/multi_map_manager/chunk_stamps','/swarm_expl/pair_opt']
            result['graph']={topic:{'publishers':[x.node_name for x in self.get_publishers_info_by_topic(topic)],
                                   'subscribers':[x.node_name for x in self.get_subscriptions_info_by_topic(topic)]} for topic in topics}
        except BaseException as exc:result['error']=str(exc);self.record('failed',error=str(exc))
        finally:
            self.phase='stop'
            for _,stop in self.commands.values():stop.publish(Empty())
            try:
                self.wait(lambda:all(d.get('native',{}).get('fsm')=='FINISH' and d.get('shadow',{}).get('stopped') for d in self.rows.values()),FLEET['stop_timeout_s'])
                result['native_stop_confirmed']=True
            except BaseException as exc:result['stop_error']=str(exc)
            result['final_rows']=self.rows
            result['all_landed_disarmed']=all(self.fresh(ns,'land') and self.fresh(ns,'px4') and d['land']['landed'] and d['px4']['arming_state']==1 for ns,d in self.rows.items())
            result['observed_arming_states']={k:sorted(v) for k,v in self.arming.items()}
            result['pass']=bool(result.get('readiness') and result.get('checks') and all(result['checks'].values()) and result.get('native_stop_confirmed') and result['all_landed_disarmed'] and 'error' not in result)
            (OUT/'result.json').write_text(json.dumps(result,indent=2))
            self.logs.close()
        print(json.dumps({k:v for k,v in result.items() if k not in ['final_rows','graph','observed_drone_states','matched_accepted_pairs']},indent=2))
        return 0 if result['pass'] else 1
if __name__=='__main__':
    rclpy.init();node=Probe()
    signal.signal(signal.SIGINT,lambda *_:setattr(node,'abort',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(node,'abort',True))
    code=node.run();node.destroy_node();rclpy.try_shutdown();raise SystemExit(code)
