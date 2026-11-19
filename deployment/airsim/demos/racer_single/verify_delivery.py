"""Evaluate the predeclared single-UAV scope and cleanup separately from task exit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

ap=argparse.ArgumentParser()
ap.add_argument('--root',type=Path,required=True)
ap.add_argument('--case',default='flight-01')
ap.add_argument('--ground-case',default='ground-01')
a=ap.parse_args();root=a.root;case=root/a.case
def read(p):return json.loads(p.read_text(encoding='utf-8'))
checks={}
result=read(case/'flight-result.json');analysis=read(case/'analysis.json')
cfg=read(root/'configuration/config.json')
checks['task']=result['pass']
checks['native_active_revisions']=len(set(result['active_revisions']))>=cfg['minimum_active_revisions']
checks['movement']=result['horizontal_displacement_m']>=cfg['minimum_observed_displacement_m']
checks['actual_references']=result['as2_references']>0 and result['px4_references']>0
checks['stop_and_ground']=all(result.get(k,False) for k in ['behavior_stop_confirmed','native_stop_acknowledged','landed','disarmed','peer_disarmed'])
checks['activity_envelope']=not analysis['active_phase_envelope_violations']
checks['ground_shadow']=read(root/a.ground_case/'ground-result.json')['pass']
checks['component_cleanup']=all(read(case/'cleanup-result.json').values())
checks['ground_cleanup']=all(read(root/a.ground_case/'cleanup-result.json').values())
checks['sensor_cleanup']=all(read(root/'sensors/cleanup-result.json').values())
children=read(case/'native-cleanup.json')
checks['native_children']=all(p['exit']==0 and not p['escalated'] for p in children)
states=read(case/'px4-ground.json')
checks['independent_px4_ground']=all(re.search(r'arming_state: 1\b',v['status']) and re.search(r'\blanded: True\b',v['land']) for v in states.values())
camera=[json.loads(line) for line in (root/'sensors/overhead.jsonl').read_text().splitlines() if line.strip()]
def pose(row):return [row['camera']['pose']['position'][k+'_val'] for k in 'xyz']
checks['overhead_updates']=len(camera)>1
# Camera RPC round-trip comparison is a coordinate check, not a flight safety gate.
checks['overhead_pose']=all(math.dist(pose(row),[row['centroid_world_ned'][0],row['centroid_world_ned'][1],row['centroid_world_ned'][2]-8.])<1e-3 for row in camera)
checks['overhead_moves']=max(math.dist(pose(row),pose(camera[0])) for row in camera)>0.01
project=read(root/'local.json')['project']
ids=subprocess.check_output(['docker','ps','-aq','--filter','label=aerohub.racer.owner='+project],text=True).split()
containers=json.loads(subprocess.check_output(['docker','inspect',*ids],text=True)) if ids else []
(root/'final-containers.json').write_text(json.dumps([{k:d[k] for k in ['Name','Id','Image','State']} for d in containers],indent=2))
checks['no_owned_process_left']=bool(containers) and all(not d['State']['Running'] for d in containers)
checks['owned_exit_codes']=bool(containers) and all(d['State']['ExitCode']==0 for d in containers)
checks['ulog_present']=(case/'flight.ulg').stat().st_size>0
core=read(root/'core-final.json')
checks['core_cleanup']=len(core)==5 and all(not r['State']['Running'] and r['State']['ExitCode']==0 for r in core)
report={'scope':'bounded native single-UAV integration; not full RACER reproduction','checks':checks,
        'pass':all(bool(v) for v in checks.values()),'overhead_samples':len(camera),
        'uncovered':['four-UAV cooperation','whole-map coverage','fault injection','RoboPi','flight collision-event statistics','hardware'],
        'note':'Passing task exit alone cannot override a failed process cleanup or missing evidence.'}
(root/'delivery-result.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['pass'] else 1)
