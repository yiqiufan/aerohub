"""Offline native-exploration evidence; no new pass thresholds or flight commands."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

parser=argparse.ArgumentParser()
parser.add_argument('--root',type=Path,required=True)
parser.add_argument('--case',required=True)
parser.add_argument('--python-deps',type=Path)
args=parser.parse_args()
root,case=args.root,args.root/args.case
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'src/tool/navigation/planner/racer/ros2_adapter/bridge'))
if args.python_deps:sys.path.insert(0,str(args.python_deps))
from trajectory import Plan, errors
from pyulog import ULog

def rows(name):
    return [json.loads(line) for line in (case/name).read_text().splitlines() if line.strip()]

events,adapter=rows('flight-events.jsonl'),rows('adapter-events.jsonl')
cfg=json.loads((root/'configuration/config.json').read_text())
result=json.loads((case/'flight-result.json').read_text())
plans={e['revision']:Plan.parse(dict(vehicle=1,received_ns=e['sim_ns'],message=e['message']))
       for e in adapter if e['event']=='native_plan_received'}
ready=next(e for e in events if e['event']=='preflight_ready')
origin=ready['airsim_ned']
observations=[e for e in events if e['event']=='airsim']
summary={'mission':result,'event_counts':dict(Counter(e['event'] for e in adapter)),
         'native_plan_count':len(plans),'stop_events':[e for e in adapter if e['event'] in
         ('stop_requested','stop_reply','action_result','stop_confirmed','stop_failed')],
         'active':[e for e in adapter if e['event']=='active'],
         'peer_observed_arming_states':sorted({e['message']['arming_state'] for e in events
             if e['event']=='state_sample' and e['topic']=='peer'}),
         'final_confirmation':[e for e in events if e['event']=='final_ground_confirmed']}
by_phase={}
samples=[]
violations=[]
peer_origin=None
peer_displacement=[]
render_differences=[]
spawn=json.loads((case/'survey.json').read_text())['settings']['Vehicles']
separation=[]
for e in observations:
    vehicles=e['source']['vehicles']
    xyz=lambda vehicle,source:[vehicles[vehicle][source]['position'][k+'_val'] for k in 'xyz']
    p,r,peer=xyz('Drone0','physics'),xyz('Drone0','rendered'),xyz('Drone1','physics')
    peer_origin=peer if peer_origin is None else peer_origin
    peer_displacement.append(math.dist(peer_origin,peer))
    render_differences.append(math.dist(p,r))
    relative=[p[0]-origin[0],p[1]-origin[1],origin[2]-p[2]]
    by_phase.setdefault(e['phase'],[]).append(relative)
    samples.append(dict(phase=e['phase'],wall_seconds=(e['wall_ns']-ready['wall_ns'])/1e9,
                        north=relative[0],east=relative[1],height=relative[2]))
    world=lambda name,pos:[pos[i]+spawn[name].get(k,0) for i,k in enumerate('XYZ')]
    dist=math.dist(world('Drone0',p),world('Drone1',peer));separation.append(dist)
    if e['phase'] in ('arming','takeoff','tracking'):
        for source,value in [('physics',p),('rendered',r)]:
            h=origin[2]-value[2]
            if not (cfg['physical_height_during_active_flight_m'][0]<=h<=cfg['physical_height_during_active_flight_m'][1]
                    and math.dist(value[:2],origin[:2])<=cfg['physical_radius_m']
                    and dist>=cfg['peer_min_separation_m']):
                violations.append(dict(wall_ns=e['wall_ns'],phase=e['phase'],source=source))
summary.update(observations=len(samples),active_phase_envelope_violations=violations,
    physics_ranges_by_phase={phase:[[min(v[i] for v in values),max(v[i] for v in values)]
                                   for i in range(3)] for phase,values in by_phase.items()},
    range_axes=['relative_north_m','relative_east_m','height_above_start_m'],
    peer_max_observed_displacement_m=max(peer_displacement),
    peer_min_observed_separation_m=min(separation),
    max_physics_render_difference_m=max(render_differences),
    max_observed_horizontal_displacement_m=max(math.hypot(s['north'],s['east']) for s in samples))
ordered=sorted(plans.values(),key=lambda p:p.start)
summary['adjacent_native_curve_handoff_differences']=[dict(before=a.id,after=b.id,
    pva_yaw_errors=errors(a.sample((b.start-a.start)/1e9),b.sample(0)))
    for a,b in zip(ordered,ordered[1:]) if 0<(b.start-a.start)/1e9<a.position.duration]
summary['native_curve_sampled_max_speed_m_s']={p.id:max(math.hypot(*p.sample(t)[1])
    for t in p.sample_times()) for p in ordered}
land=next(e for e in events if e['event']=='action_requested' and e['action']=='land_success')
land_result=next((e for e in events if e['event']=='action_result' and e['action']=='land_success'),None)
landed=next((e for e in events if e['event']=='recovery_telemetry' and e['topic']=='land_detected'
             and e['message']['landed'] and e['wall_ns']>land['wall_ns']),None)
disarmed=next((e for e in events if e['event']=='recovery_telemetry' and e['topic']=='px4'
               and e['message']['arming_state']==1 and e['wall_ns']>land['wall_ns']),None)
summary['seconds_since_land_request_wall_clock']={k:(e['wall_ns']-land['wall_ns'])/1e9 if e else None
    for k,e in [('land_action_result',land_result),('landed',landed),('disarmed',disarmed)]}
ulog=case/'flight.ulg'

u=ULog(str(ulog))
messages=[dict(timestamp_us=m.timestamp,text=m.message) for m in u.logged_messages]
ld=u.get_dataset('vehicle_land_detected').data
transitions=[];previous=None
for i in range(len(ld['timestamp'])):
    row={k:v[i].item() for k,v in ld.items()}
    state=tuple(row[k] for k in ('ground_contact','maybe_landed','landed'))
    if state!=previous:transitions.append(row);previous=state
summary['ulog']={'firmware':u.msg_info_dict.get('ver_sw'),'sha256':hashlib.sha256(ulog.read_bytes()).hexdigest(),
    'dropouts':[dict(timestamp_us=d.timestamp,duration_ms=d.duration) for d in u.dropouts],
    'messages':messages,'land_transitions':transitions}
summary['native_cleanup']=json.loads((case/'native-cleanup.json').read_text())
summary['limitations']=['20-second bounded native exploration; no complete-map coverage or four-vehicle integration',
    'No flight-time collision-event polling/statistics; scene collision remained enabled',
    'Post-stop/landing physical samples diagnostic, not subject to active-flight envelope gate',
    'AirSim sequential RPC observations not synchronized; no unseen interval guarantee',
    'Curve sampling diagnostics do not prove continuous-time maxima; no new post-hoc threshold',
    'No hardware, RoboPi or fault injection acceptance in this run']
(case/'analysis.json').write_text(json.dumps(summary,indent=2))
(case/'trajectory-samples.json').write_text(json.dumps(samples,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k in ('mission','native_plan_count','event_counts',
    'peer_max_observed_displacement_m','peer_min_observed_separation_m','active_phase_envelope_violations',
    'seconds_since_land_request_wall_clock','native_curve_sampled_max_speed_m_s','native_cleanup')},indent=2))
