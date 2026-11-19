"""Original RACER FSM and two real solvers for one configured fleet member."""
import json,math,os,re,runpy,signal,subprocess,time
from threading import Thread,Lock
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from common import FLEET,VEHICLE,NS,PREFIX,CONFIG,OUT
from native_lifecycle import NativeLifecycle

diagnostic_config=CONFIG.parent/'diagnostics.json'
diagnostics=json.loads(diagnostic_config.read_text()) if diagnostic_config.exists() else {}
if diagnostics.get('enabled'):
    os.environ['RACER_DIAGNOSTIC_FILE']=str(OUT/'map-query.jsonl')
identity=VEHICLE['racer_id']
params=runpy.run_path('/component/native/src/exploration_manager/launch/racer_pillar.launch.py')['planner_params'](identity,False,1.,False)
mapping=yaml.safe_load((CONFIG/'mapper.yaml').read_text())['/**']['ros__parameters']
for section in ('sdf_map','map_ros'):params.update({section+'.'+k:v for k,v in mapping[section].items()})
solver=OUT/'solver-work';solver.mkdir()
params.update({'use_sim_time':True,'exploration.drone_num':FLEET['native_slots'],
    'exploration.tsp_dir':str(solver),'exploration.mtsp_dir':str(solver),
    'partitioning.use_swarm_tf':False,'map_ros.report_coverage':False,
    'map_ros.coverage_report_interval':1.,'visualization.show_grid_text':False,
    'external_execution.enabled':True,'external_execution.lead':.75,'external_execution.stop_reserve':.35,
    'perception_utils.top_angle':math.atan(240/320),'perception_utils.left_angle':math.pi/4,
    'perception_utils.right_angle':math.pi/4})
# Explicit opt-in; no additional decision logging for existing configurations.
params['frontier.decision_diagnostics']=bool(diagnostics.get('enabled') and diagnostics.get('frontier_decisions',False))
for axis,bounds in zip('xyz',FLEET['planning_box_enu']):params['sdf_map.box_min_'+axis],params['sdf_map.box_max_'+axis]=bounds
for section,v,a in [('exploration','vm','am'),('manager','max_vel','max_acc'),('search','max_vel','max_acc'),('optimization','max_vel','max_acc'),('bspline','limit_vel','limit_acc')]:
    params[section+'.'+v]=.65;params[section+'.'+a]=.9
# ROS YAML cannot infer an empty sequence's type. Omit it and let the C++
# declaration use its typed empty vector; trajectory queries remain enabled.
if diagnostics.get('enabled') and diagnostics.get('probe_points'):
    params['diagnostic.probe_points']=[float(x) for x in diagnostics['probe_points']]
(OUT/'native-params.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=rclpy.create_node('racer_native_supervisor_'+NS)
pub=node.create_publisher(String,PREFIX+'/native_state',10)
state={'fsm':'STARTING','frontier_points':0,'view_selections':0,'process_failure':'','racer_id':identity}
lifecycle=NativeLifecycle(identity)
state.update(lifecycle.feed(''))
frontiers={};lock=Lock();processes=[];readers=[]
def marker(msg):
    if msg.ns=='frontier':
        frontiers[msg.id]=len(msg.points)
        with lock:state['frontier_points']=sum(frontiers.values())
node.create_subscription(Marker,PREFIX+'/frontier',marker,10)
def launch(name,args):
    proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
    processes.append((name,proc))
    def reader():
        with (OUT/(name+'.log')).open('x',buffering=1) as f:
            for line in proc.stdout:
                f.write(line)
                with lock:
                    if name=='exploration':state.update(lifecycle.feed(line))
                    found=re.search(r'Drone '+str(identity)+r' from \w+ to (\w+)',line)
                    if found:state['fsm']=found.group(1)
                    if 'Next view:' in line:state['view_selections']+=1
                    if 'get response 1' in line:state['accepted_pair_responses']=state.get('accepted_pair_responses',0)+1
                    activated=re.search(r'handoff active id=(\d+)',line)
                    if activated:state['last_activated_revision']=int(activated.group(1))
                    if 'handoff status received:' in line:state['execution_status_received']=state.get('execution_status_received',0)+1
                    if 'handoff status applied:' in line:
                        state['execution_status_applied']=state.get('execution_status_applied',0)+1
                        delay=re.search(r'queue_ms=([0-9.]+)',line)
                        if delay:state['max_execution_status_queue_ms']=max(state.get('max_execution_status_queue_ms',0),float(delay.group(1)))
                    if 'path failure:' in line:state['last_path_failure']=line.strip()
    t=Thread(target=reader);t.start();readers.append(t)
for problem in (1,2):
    launch('solver-'+str(problem),['/racer-run/install/lkh_mtsp_solver/lib/lkh_mtsp_solver/mtsp_node','--ros-args',
        '-r','__node:=solver_'+NS+'_'+str(problem),'-p','exploration.drone_id:='+str(identity),
        '-p','exploration.problem_id:='+str(problem),'-p','exploration.mtsp_dir:='+str(solver)])
args=['/racer-run/install/exploration_manager/lib/exploration_manager/exploration_node','--ros-args',
      '-r','__node:=racer_'+NS,'--params-file',str(OUT/'native-params.yaml')]
remaps={'/odom_world':PREFIX+'/odometry','/map_ros/depth':PREFIX+'/depth','/map_ros/pose':PREFIX+'/camera_pose',
    '/map_ros/cloud':PREFIX+'/unused_cloud','/move_base_simple/goal':PREFIX+'/trigger',
    '/sdf_map/occupancy_all':PREFIX+'/occupancy','/sdf_map/occupancy_local':PREFIX+'/occupancy_local',
    '/planning_vis/frontier':PREFIX+'/frontier','/planning_vis/viewpoints':PREFIX+'/viewpoints',
    '/planning_vis/trajectory':PREFIX+'/trajectory','/planning/bspline':PREFIX+'/legacy_bspline',
    '/planning/replan':PREFIX+'/legacy_replan','/planning/new':PREFIX+'/legacy_new'}
for stem in ('/swarm_expl/drone_state','/swarm_expl/pair_opt','/swarm_expl/pair_opt_res',
             '/planning/swarm_traj','/multi_map_manager/chunk_stamps','/multi_map_manager/chunk_data'):
    remaps[stem+'_send']=remaps[stem+'_recv']=stem
for stem in ('/swarm_expl/grid_tour','/swarm_expl/hgrid'):remaps[stem+'_send']=stem
for key in ('occupancy_local_inflate','depth_cloud','unknown','esdf','update_range'):
    remaps['/sdf_map/'+key]=PREFIX+'/'+key
for source,target in remaps.items():args+=['-r',source+':='+target]
(OUT/'native-command.json').write_text(json.dumps(args,indent=2))
launch('exploration',args)
running=True
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def publish():
    with lock:row=dict(state)
    pub.publish(String(data=json.dumps(row)))
node.create_timer(.1,publish)
while running:
    rclpy.spin_once(node,timeout_sec=.1)
    for name,p in processes:
        if p.poll() is not None:
            with lock:state['process_failure']=name+' exited '+str(p.returncode)
cleanup=[]
for name,p in reversed(processes):
    escalated=False
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            escalated=True;os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    cleanup.append(dict(name=name,pid=p.pid,exit=p.returncode,escalated=escalated))
for t in readers:t.join(timeout=3)
(OUT/'native-cleanup.json').write_text(json.dumps(cleanup,indent=2))
node.destroy_node();rclpy.try_shutdown()
raise SystemExit(0 if all(p['exit']==0 and not p['escalated'] for p in cleanup) else 1)
