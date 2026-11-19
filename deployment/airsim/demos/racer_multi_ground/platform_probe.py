"""Read-only simulator identity and camera audit; no collision-event queries."""
import json,os
from pathlib import Path
from uav_as2_sensor_adapter.rpc import Rpc
fleet=json.loads(Path('/config/fleet.json').read_text());root=Path('/result')
rpc=Rpc(os.environ['AIRSIM_HOST'],int(os.environ['AIRSIM_RPC_PORT']),2.)
vehicles=rpc.call('listVehicles');settings=json.loads(rpc.call('getSettingsString'))
expected=[v['airsim'] for v in fleet['vehicles']]
if sorted(vehicles)!=sorted(expected):raise RuntimeError('UE did not load expected fleet: '+str(vehicles))
rows={}
for v in fleet['vehicles']:
    actual=settings['Vehicles'][v['airsim']]
    if actual['VehicleSysID']!=v['px4_system_id'] or [actual[k] for k in ('X','Y','Z')]!=v['spawn_ned']:raise ValueError('settings mapping mismatch')
    rows[v['airsim']]={'pose':rpc.call('simGetVehiclePose',v['airsim']),
                       'camera':rpc.call('simGetCameraInfo','front_center',v['airsim'],False)}
(root/'runtime-settings.json').write_text(json.dumps(settings,indent=2))
(root/'platform.json').write_text(json.dumps(rows,indent=2))
print('Loaded real fleet:',vehicles)
