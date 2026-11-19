"""Read-only survey; no collision-event polling and no movement RPC."""
import json
import math
import os
from pathlib import Path
import sys
from importlib import import_module
from uav_as2_sensor_adapter.rpc import Rpc

from geodesy import ned_to_geo as geo
rpc = Rpc(os.environ['AIRSIM_HOST'],int(os.environ.get("AIRSIM_RPC_PORT",41461)),3)
settings = json.loads(rpc.call('getSettingsString'))
poses = [rpc.call('simGetVehiclePose',v) for v in ('Drone0','Drone1')]
origin = {k.lower():settings.get('OriginGeopoint',{}).get(k,v) for k,v in
          [('Latitude',47.641468),('Longitude',-122.140165),('Altitude',122.0)]}
p = [poses[0]['position'][k+'_val']+settings['Vehicles']['Drone0'].get(k.upper(),0) for k in 'xyz']
def point(n,e,h):
    return geo(origin,p[0]+n,p[1]+e,p[2]-h)
rays=[]
for h in (.4,.75,1.2,1.8):
    for degrees in range(0,360,30):
        a=math.radians(degrees)
        for radius in (1.,2.,3.,5.):
            clear=rpc.call('simTestLineOfSightBetweenPoints',point(0,0,h),point(radius*math.cos(a),radius*math.sin(a),h))
            rays.append(dict(height=h,angle_deg=degrees,radius=radius,clear=clear))
takeoff=[]
for n,e in [(0,0),(.35,0),(-.35,0),(0,.35),(0,-.35)]:
    takeoff.append(rpc.call('simTestLineOfSightBetweenPoints',point(n,e,.3),point(n,e,1.2)))
out=dict(settings=settings,vehicle_pose_before=poses,
         vehicle_pose_after=[rpc.call('simGetVehiclePose',v) for v in ('Drone0','Drone1')],
         rays=rays,takeoff=takeoff,pass_takeoff=all(takeoff),
         limitation='Sampled LOS survey is not exhaustive free-volume or collision-event proof')
Path('/result/survey.json').write_text(json.dumps(out,indent=2))
print(json.dumps(dict(takeoff=all(takeoff),clear=sum(r['clear'] for r in rays),rays=len(rays))))
