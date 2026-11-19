"""Run the existing native exploration FSM and its real solvers, with bounded cleanup."""
import json
import math
import os
from pathlib import Path
import re
import runpy
import signal
import subprocess
from threading import Thread, Lock
import time
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import yaml

root=Path('/result')
config=json.loads(Path('/config/config.json').read_text())
params=runpy.run_path('/component/native/src/exploration_manager/launch/racer_pillar.launch.py')['planner_params'](1,False,1.,False)
mapping=yaml.safe_load(Path('/config/mapper.yaml').read_text())['/**']['ros__parameters']
for section in ('sdf_map','map_ros'):
    params.update({section+'.'+k:v for k,v in mapping[section].items()})
solver=root/'solver-work'
solver.mkdir()
params.update({'use_sim_time':True,'exploration.drone_num':1,'exploration.tsp_dir':str(solver),
    'exploration.mtsp_dir':str(solver),'partitioning.use_swarm_tf':False,
    'map_ros.report_coverage':False,'map_ros.coverage_report_interval':1.,'visualization.show_grid_text':False,
    'external_execution.enabled':True,'external_execution.lead':config['handoff_lead_seconds'],
    'external_execution.stop_reserve':config['handoff_stop_reserve_seconds'],
    'perception_utils.top_angle':math.atan(240/320),
    'perception_utils.left_angle':math.pi/4,'perception_utils.right_angle':math.pi/4})
for axis,bounds in zip('xyz',config['planning_box_enu']):
    params['sdf_map.box_min_'+axis],params['sdf_map.box_max_'+axis]=bounds
for section,v,a in [('exploration','vm','am'),('manager','max_vel','max_acc'),
                    ('search','max_vel','max_acc'),('optimization','max_vel','max_acc'),('bspline','limit_vel','limit_acc')]:
    params[section+'.'+v]=config['planner_speed_m_s']
    params[section+'.'+a]=config['planner_acceleration_m_s2']
(root/'native-params.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
rclpy.init()
node=rclpy.create_node('racer_native_exploration_supervisor')
pub=node.create_publisher(String,'/racer_explore/native_state',10)
state={'fsm':'STARTING','frontier_points':0,'view_selections':0,'native_active':[], 'process_failure':''}
lock=Lock()
frontiers={}
def on_marker(msg):
    if msg.ns=='frontier':
        frontiers[msg.id]=len(msg.points)
        with lock: state['frontier_points']=sum(frontiers.values())
node.create_subscription(Marker,'/racer_explore/frontier',on_marker,10)
processes=[]
readers=[]
def launch(name,args):
    p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
    processes.append((name,p))
    def read():
        with (root/(name+'.log')).open('x',buffering=1) as f:
            for line in p.stdout:
                f.write(line)
                with lock:
                    m=re.search(r'Drone 1 from \w+ to (\w+)',line)
                    if m: state['fsm']=m.group(1)
                    if 'Next view:' in line: state['view_selections']+=1
                    m=re.search(r'handoff active id=(\d+)',line)
                    if m: state['native_active'].append(int(m.group(1)))
                    if 'handoff stop confirmed:' in line: state['last_stop']=line.strip()
    t=Thread(target=read);t.start();readers.append(t)
for problem in (1,2):
    launch('solver-'+str(problem),['/racer-run/install/lkh_mtsp_solver/lib/lkh_mtsp_solver/mtsp_node',
      '--ros-args','-p','exploration.drone_id:=1','-p',f'exploration.problem_id:={problem}',
      '-p',f'exploration.mtsp_dir:={solver}'])
args=['/racer-run/install/exploration_manager/lib/exploration_manager/exploration_node','--ros-args',
      '--params-file',str(root/'native-params.yaml')]
remaps={'/odom_world':'/racer_short/odometry','/map_ros/depth':'/racer_ground/depth',
 '/map_ros/pose':'/racer_ground/camera_pose','/map_ros/cloud':'/racer_ground/unused_cloud',
 '/sdf_map/occupancy_all':'/racer_ground/occupancy', '/move_base_simple/goal':'/racer_explore/trigger',
 '/planning_vis/frontier':'/racer_explore/frontier','/planning_vis/viewpoints':'/racer_explore/viewpoints',
 '/planning_vis/trajectory':'/racer_explore/trajectory'}
for a,b in remaps.items(): args+=['-r',a+':='+b]
launch('exploration',args)
running=True
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
def publish():
    with lock: row=dict(state)
    row['wall_ns']=time.time_ns()
    pub.publish(String(data=json.dumps(row)))
node.create_timer(.1,publish)
while running:
    rclpy.spin_once(node,timeout_sec=.1)
    for name,p in processes:
        if p.poll() is not None:
            with lock: state['process_failure']=f'{name} exited {p.returncode}'
cleanup=[]
for name,p in reversed(processes):
    escalated=False
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            escalated=True;os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=3)
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    cleanup.append(dict(name=name,pid=p.pid,exit=p.returncode,escalated=escalated))
for t in readers:t.join(timeout=3)
(root/'native-cleanup.json').write_text(json.dumps(cleanup,indent=2))
node.destroy_node();rclpy.try_shutdown()
raise SystemExit(0 if all(p['exit']==0 and not p['escalated'] for p in cleanup) else 1)
