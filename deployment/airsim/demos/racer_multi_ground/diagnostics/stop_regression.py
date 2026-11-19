"""Synthetic scheduling regression against the real RACER binary; never a flight test."""
import json,os,signal,subprocess,time,argparse
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from exploration_manager.msg import TrajectoryExecution
from bspline.msg import Bspline

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--mode',choices=['busy-stop','busy-shutdown','accept-stop'],default='busy-stop')
mode=ap.parse_args().mode
out=Path('/result');child=out/'native';child.mkdir()
env=dict(os.environ,RACER_OUTPUT=str(child),RACER_VEHICLE_INDEX='0')
log=(out/'supervisor.log').open('w')
proc=subprocess.Popen(['python3','/multi/native.py'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=rclpy.create_node('synthetic_stop_regression')
latest={};events=[];accepted=[];active=[]
n.create_subscription(String,'/racer_multi/drone0/native_state',lambda m:latest.update(json.loads(m.data)),10)
pubs={k:n.create_publisher(t,topic,10) for k,t,topic in [
    ('clock',Clock,'/clock'),('odom',Odometry,'/racer_multi/drone0/odometry'),
    ('pose',PoseStamped,'/racer_multi/drone0/camera_pose'),('depth',Image,'/racer_multi/drone0/depth'),
    ('trigger',PoseStamped,'/racer_multi/drone0/trigger'),('status',TrajectoryExecution,'/racer_execution/drone_1/status')]}
def plan(msg):
    if mode=='accept-stop':
        accepted.append(msg.traj_id)
        pubs['status'].publish(TrajectoryExecution(traj_id=msg.traj_id,status=1,detail='synthetic acceptance, no physical execution'))
def shared(msg):
    if msg.drone_id==1 and msg.traj_id in accepted:active.append(msg.traj_id)
n.create_subscription(Bspline,'/racer_execution/drone_1/plan',plan,10)
n.create_subscription(Bspline,'/planning/swarm_traj',shared,10)
start=time.monotonic();next_depth=0
def tick():
    global next_depth
    stamp=n.get_clock().now().to_msg();pubs['clock'].publish(Clock(clock=stamp))
    od=Odometry();od.header.stamp=stamp;od.header.frame_id='racer_world';od.child_frame_id='racer_world'
    # Deliberately below the planning box to keep the real solver in retries.
    od.pose.pose.position.z=.7 if mode=='accept-stop' else -1.
    od.pose.pose.orientation.w=1.;pubs['odom'].publish(od)
    if time.monotonic()>=next_depth:
        next_depth=time.monotonic()+.5
        p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='racer_world'
        p.pose.position.y=.46;p.pose.position.z=1.
        p.pose.orientation.x=-2**-.5;p.pose.orientation.w=2**-.5
        img=Image();img.header=p.header;img.height=480;img.width=640;img.encoding='16UC1';img.step=1280
        img.data=bytes([0xa0,0x0f])*640*480
        pubs['pose'].publish(p);pubs['depth'].publish(img)
n.create_timer(.02,tick)
def wait(pred,seconds):
    deadline=time.monotonic()+seconds
    while not pred():
        if time.monotonic()>deadline:raise TimeoutError(str(latest))
        if proc.poll() is not None:raise RuntimeError('native supervisor exited '+str(proc.returncode))
        rclpy.spin_once(n,timeout_sec=.02)
result={'scope':'synthetic real-native scheduling regression; no PX4/AS2/flight','mode':mode,'pass':False}
try:
    wait(lambda:latest.get('fsm')=='WAIT_TRIGGER' and latest.get('frontier_points',0)>0,30)
    pubs['trigger'].publish(PoseStamped())
    wait(lambda:(latest.get('last_activated_revision') in accepted and bool(active)) if mode=='accept-stop' else latest.get('view_selections',0)>=10,20)
    result['before_stop']=dict(latest);sent=time.monotonic()
    if mode!='busy-shutdown':
        pubs['status'].publish(TrajectoryExecution(status=4,detail='synthetic busy-planner stop regression'))
        try:
            wait(lambda:latest.get('fsm')=='FINISH',8)
            result.update(stop_confirmed=True,stop_wall_seconds=time.monotonic()-sent)
            end=time.monotonic()+1.;wait(lambda:time.monotonic()>end,2)
            result['remains_finished']=latest.get('fsm')=='FINISH'
        except TimeoutError as exc:result['stop_error']=str(exc)
    result['after_stop']=dict(latest)
except Exception as exc:result['error']=str(exc)
finally:
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=20)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    result['supervisor_exit']=proc.returncode
    p=child/'native-cleanup.json'
    result['children']=json.loads(p.read_text()) if p.exists() else None
    result['synthetic_accepted_ids']=accepted;result['shared_ids_matching_synthetic_ack']=active
    behavior_ok=bool(result.get('stop_confirmed') and result.get('remains_finished')) if mode!='busy-shutdown' else 'before_stop' in result
    result['pass']=bool(behavior_ok and proc.returncode==0 and 'error' not in result)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    n.destroy_node();rclpy.try_shutdown();log.close()
print(json.dumps(result,indent=2));raise SystemExit(0 if result['pass'] else 1)
