"""Read-only per-UAV takeoff columns; no collision-event query or motion API."""
import os,json
from pathlib import Path
from uav_as2_sensor_adapter.rpc import Rpc
from geodesy import ned_to_geo
rpc=Rpc(os.environ['AIRSIM_HOST'],int(os.environ['AIRSIM_RPC_PORT']),3.)
fleet=json.loads(Path('/config/fleet.json').read_text());settings=json.loads(rpc.call('getSettingsString'))
origin={k.lower():settings.get('OriginGeopoint',{}).get(k,v) for k,v in [('Latitude',47.641468),('Longitude',-122.140165),('Altitude',122.)]}
result={}
for v in fleet['vehicles']:
    pose=rpc.call('simGetVehiclePose',v['airsim']);p=[pose['position'][k+'_val']+v['spawn_ned'][i] for i,k in enumerate('xyz')]
    columns=[]
    for n,e in [(0,0),(.35,0),(-.35,0),(0,.35),(0,-.35)]:
        columns.append(rpc.call('simTestLineOfSightBetweenPoints',ned_to_geo(origin,p[0]+n,p[1]+e,p[2]-.3),ned_to_geo(origin,p[0]+n,p[1]+e,p[2]-1.2)))
    result[v['airsim']]=dict(pose_before=pose,pose_after=rpc.call('simGetVehiclePose',v['airsim']),takeoff_columns=columns,pass_takeoff=all(columns))
Path('/result/survey.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2));raise SystemExit(0 if all(r['pass_takeoff'] for r in result.values()) else 1)
