"""Reuse the fleet camera helper; account for AirSim per-vehicle pose origins."""
import copy
import json
import os
from pathlib import Path
import signal
import time
from deployment.swarm.overhead_view import update_camera
from deployment.swarm.view import camera_options
from uav_as2_sensor_adapter.rpc import Rpc

rpc=Rpc(os.environ['AIRSIM_HOST'],int(os.environ.get('AIRSIM_RPC_PORT',41461)),1.0)
settings=json.loads(rpc.call('getSettingsString'))
camera=camera_options({'altitude_m':8.,'update_hz':3.})
if camera['name'] not in settings.get('ExternalCameras',{}):raise RuntimeError('shared external camera not loaded; refresh scene settings')

class WorldPoses:
    def call(self,method,*args):
        result=rpc.call(method,*args)
        if method=='simGetVehiclePose':
            result=copy.deepcopy(result)
            for axis in 'xyz':result['position'][axis+'_val']+=settings['Vehicles'][args[0]].get(axis.upper(),0.)
        return result

running=True
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
with Path('/result/overhead.jsonl').open('x',buffering=1) as f:
    while running:
        center=update_camera(WorldPoses(),list(settings['Vehicles']),camera)
        info=rpc.call('simGetCameraInfo',camera['name'],'',True)
        f.write(json.dumps({'wall_ns':time.time_ns(),'centroid_world_ned':center,'camera':info})+'\n')
        time.sleep(1/camera['update_hz'])
