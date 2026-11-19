"""PC multi-RACER ground-only entry. No arming or flight command is implemented."""
import argparse,hashlib,json,re,shutil,subprocess,sys,time
from pathlib import Path
DEMO=Path(__file__).resolve().parent;REPO=DEMO.parents[3];SINGLE=DEMO.parent/'racer_single'
MULTI=REPO/'src/tool/navigation/planner/racer/ros2_adapter/multi'
ROS='source /opt/ros/humble/setup.bash\n'
ONBOARD=ROS+'source /root/aerostack2_ws/install/setup.bash\nsource /opt/px4_ws/install/setup.bash\nsource /opt/uav_execution_ws/install/setup.bash\n'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
def call(args,timeout=None):return subprocess.check_output(args,text=True,encoding='utf-8',errors='replace',timeout=timeout).strip()
class Deployment:
    def __init__(self,path):
        self.local=read(path);self.root=Path(self.local['run_root']).resolve();self.release=Path(self.local['release']).resolve()
        self.prefix=self.local['project'];self.fleet_path=self.config_path(path,'fleet_config',DEMO/'fleet.json')
        if 'fleet_config' in self.local:self.local['fleet_config']=str(self.fleet_path)
        self.fleet=read(self.fleet_path);self.fleet_source_sha=sha(self.fleet_path);self.images=read(SINGLE/'images.lock.json')
        if not re.fullmatch('[a-z][a-z0-9-]{2,48}',self.prefix):raise ValueError('invalid project')
        ids=[v['racer_id'] for v in self.fleet['vehicles']]
        if ids!=list(range(1,len(ids)+1)) or self.fleet['native_slots']!=len(ids)+1:raise ValueError('RACER IDs/ground map slot invalid')
        self.compose=['docker','compose','-p',self.prefix,'-f',str(self.root/'compose.json')]
    def config_path(self,local_path,key,default):
        value=Path(self.local[key]) if key in self.local else default
        return (value if value.is_absolute() else Path(local_path).resolve().parent/value).resolve()
    def core(self,role):
        container=call(self.compose+['ps','-q',role],timeout=10)
        if not container:raise RuntimeError(f'{role} has no running container; inspect startup-result.json and container logs')
        return container
    def ground(self,out):
        states={}
        for v in self.fleet['vehicles']:
            container=self.core(v['namespace']);cmd=['docker','exec',container,'/opt/px4-gazebo/bin/px4-listener','--instance',str(v['px4_instance'])]
            states[v['namespace']]={'status':call(cmd+['vehicle_status','-n','1'],timeout=10),'land':call(cmd+['vehicle_land_detected','-n','1'],timeout=10)}
        (out/'px4-ground.json').write_text(json.dumps(states,indent=2))
        if not all(re.search(r'arming_state: 1\b',s['status']) and re.search(r'\blanded: True\b',s['land']) for s in states.values()):raise RuntimeError('fleet ground confirmation missing; preserve flight services')
    def up(self):
        if sha(self.fleet_path)!=self.fleet_source_sha:raise ValueError('selected fleet configuration changed')
        self.root.mkdir(parents=True,exist_ok=False)
        build=read(self.release/'manifest.json')
        if build['status']!='built' or build['images']!=self.images:raise ValueError('build/image lock mismatch')
        for rel,h in build['source_files'].items():
            if '/native/' in rel or '/workstation_interfaces/' in rel:
                if sha(REPO/rel)!=h:raise ValueError('built source mismatch '+rel)
        for rel,h in build['artifacts'].items():
            if sha(self.release/rel)!=h:raise ValueError('artifact mismatch '+rel)
        for rel,h in read(SINGLE/'dependencies.lock.json')['files'].items():
            if sha(self.release/'deps'/rel)!=h:raise ValueError('dependency mismatch '+rel)
        shutil.copyfile(self.local['settings'],self.root/'settings.json')
        shutil.copyfile(self.release/'manifest.json',self.root/'build-manifest.json')
        (self.root/'local.json').write_text(json.dumps(self.local,indent=2))
        (self.root/'fleet-input.json').write_text(json.dumps(self.fleet,indent=2))
        (self.root/'configuration-sources.json').write_text(json.dumps({'fleet':{'path':str(self.fleet_path),'sha256':self.fleet_source_sha}},indent=2))
        args=['docker','run','--rm','--network','none','--pull=never','-e','PYTHONPATH=/repo']
        for src,dst,ro in [(REPO,'/repo',True),(DEMO,'/code',True),(SINGLE,'/single',True),(self.root,'/result',False)]:
            args+=['--mount',f'type=bind,source={src.as_posix()},target={dst}'+(',readonly' if ro else '')]
        subprocess.run(args+['--entrypoint','python3',self.images['onboard'],'/code/prepare_config.py'],check=True)
        diagnostics=self.local.get('diagnostics',{})
        if diagnostics.get('enabled'):
            import math
            points=diagnostics.get('probe_points',[])
            if len(points)%3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in points):raise ValueError('diagnostic probe_points must be finite XYZ triples')
            if not isinstance(diagnostics.get('queue_messages',128),int) or diagnostics.get('queue_messages',128)<1:raise ValueError('queue_messages must be positive')
            (self.root/'configuration/diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
        services={};common={'AIRSIM_HOST':self.local['airsim_host'],'AIRSIM_RPC_PORT':str(self.local['rpc_port']),
            'ROS_DOMAIN_ID':str(self.local['domain']),'FLIGHT_BOARD_SENSORS':'0','FLIGHT_BOARD_CAMERAS':'0'}
        def volume(src,dst):return {'type':'bind','source':str(src),'target':dst,'read_only':True}
        for v in self.fleet['vehicles']:
            ns=v['namespace'];cfg=self.root/'configuration'/ns
            services[ns]={'image':self.images['flight'],'pull_policy':'never','init':True,'command':['vehicle'],
                'environment':dict(common,VEHICLE_NAME=v['airsim'],VEHICLE_NAMESPACE=ns,PX4_INSTANCE=str(v['px4_instance']),
                                   PX4_SYSTEM_ID=str(v['px4_system_id']),XRCE_AGENT_PORT=str(v['xrce_port'])),
                'ports':[f'127.0.0.1:{14583+v["index"]}:{14583+v["index"]}/udp'],
                'volumes':[volume(self.root/'settings.json','/config/settings.json')]}
            services['as2-'+ns]={'image':self.images['onboard'],'pull_policy':'never','init':True,
                'environment':{'ROS_DOMAIN_ID':str(self.local['domain']),'AS2_FLEET_FILE':'/config/fleet.yaml','AS2_EXECUTION_BACKEND':'follow_path'},
                'volumes':[volume(cfg/'fleet.yaml','/config/fleet.yaml')]}
            services['navigation-'+ns]={'image':self.images['onboard'],'pull_policy':'never','init':True,'stop_signal':'SIGINT',
                'environment':{'ROS_DOMAIN_ID':str(self.local['domain'])},'volumes':[volume(cfg/'navigation.yaml','/config/navigation.yaml')],
                'command':['/root/aerostack2_ws/install/lib/as2_navigation_inputs/navigation_inputs_node','--ros-args','-r','__ns:=/'+ns,'--params-file','/config/navigation.yaml']}
        services['clock']={'image':self.images['flight'],'pull_policy':'never','init':True,'command':['clock'],
            'environment':dict(common,VEHICLE_NAME=self.fleet['vehicles'][0]['airsim'])}
        composition={'services':services}
        if self.local.get('docker_subnet'):
            import ipaddress
            network=ipaddress.ip_network(self.local['docker_subnet'],strict=True)
            if network.version!=4 or not network.is_private:raise ValueError('docker_subnet must be a private IPv4 network')
            ids=call(['docker','network','ls','-q']).splitlines()
            existing=json.loads(call(['docker','network','inspect']+ids)) if ids else []
            (self.root/'network-before.json').write_text(json.dumps(existing,indent=2))
            for item in existing:
                for cfg in item.get('IPAM',{}).get('Config') or []:
                    if cfg.get('Subnet') and network.overlaps(ipaddress.ip_network(cfg['Subnet'])):raise ValueError('docker_subnet overlaps '+item['Name'])
            composition['networks']={'default':{'ipam':{'config':[{'subnet':str(network)}]}}}
        (self.root/'compose.json').write_text(json.dumps(composition,indent=2))
        self.start_core()
    def start_core(self, timeout=60):
        """Start flight services first; never restart or arm a vehicle on failure."""
        flight = ['clock'] + [v['namespace'] for v in self.fleet['vehicles']]
        compute = [role+'-'+v['namespace'] for role in ['as2','navigation'] for v in self.fleet['vehicles']]
        stages = []
        report = {'pass': False, 'readiness_timeout_s': timeout, 'stages': stages}
        def mark(stage, **fields):
            stages.append(dict(stage=stage, wall_ns=time.time_ns(), **fields))
            (self.root/'startup-result.json').write_text(json.dumps(report,indent=2))
        try:
            mark('flight-start', services=flight)
            subprocess.run(self.compose+['up','-d']+flight,check=True,timeout=60)
            deadline=time.monotonic()+timeout
            last_error='not ready'
            while True:
                for role in flight:
                    info=read_inspect(self.prefix+'-'+role+'-1',timeout=10)
                    if not info['State']['Running']:
                        raise RuntimeError(f'{role} exited during startup: {info["State"]["ExitCode"]}')
                try:
                    self.ground(self.root)
                    connected=True
                    for v in self.fleet['vehicles']:
                        status=call(['docker','exec',self.prefix+'-'+v['namespace']+'-1',
                            '/opt/px4-gazebo/bin/px4-uxrce_dds_client','--instance',str(v['px4_instance']),'status'],timeout=10)
                        (self.root/(v['namespace']+'-startup-dds.log')).write_text(status)
                        connected=connected and 'Running, connected' in status
                    if connected:break
                    last_error='DDS not connected'
                except (subprocess.CalledProcessError,subprocess.TimeoutExpired,RuntimeError) as exc:
                    last_error=str(exc)
                if time.monotonic()>=deadline:
                    raise RuntimeError('flight readiness timeout: '+last_error)
                time.sleep(1)
            mark('flight-ready')
            mark('compute-start', services=compute)
            subprocess.run(self.compose+['up','-d']+compute,check=True,timeout=60)
            report['pass']=True
            # This is dependency startup, not proof of AS2 task readiness or flight success.
            mark('services-started')
        except Exception as exc:
            report['error']=str(exc)
            report['recovery']='Services retained. Inspect actual PX4 state and logs before stopping owned resources; no automatic restart.'
            mark('failed')
            snapshots=[]
            for role in flight+compute:
                name=self.prefix+'-'+role+'-1'
                try:
                    info=read_inspect(name,timeout=10)
                    snapshots.append({k:info[k] for k in ['Name','Image','State']})
                    with (self.root/(role+'-startup.log')).open('w',encoding='utf-8') as f:
                        subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT,timeout=10)
                except Exception as capture_error:
                    snapshots.append({'Name':name,'capture_error':str(capture_error)})
            (self.root/'startup-failure-containers.json').write_text(json.dumps(snapshots,indent=2))
            raise

    def run(self,role,image,script,detached=True):
        out=self.root/role;out.mkdir(exist_ok=False)
        args=['docker','run','--init','--pull=never','--no-healthcheck','--name',self.prefix+'-'+role,
              '--label','aerohub.racer.owner='+self.prefix,'--network',self.prefix+'_default',
              '-e','ROS_DOMAIN_ID='+str(self.local['domain']),'-e','AIRSIM_HOST='+self.local['airsim_host'],
              '-e','AIRSIM_RPC_PORT='+str(self.local['rpc_port']),'-e','PYTHONDONTWRITEBYTECODE=1',
              '-e','ROS_LOG_DIR=/result/ros-logs','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds_udp.xml',
              '-e','PYTHONPATH=/multi:/component/bridge:/sensors:/repo']
        mounts={REPO:'/repo',DEMO:'/code',SINGLE:'/single',MULTI:'/multi',REPO/'src/tool/navigation/planner/racer/ros2_adapter':'/component',
                self.root/'configuration':'/config',self.release/'native':'/racer-run',self.release/'interfaces':'/interfaces',
                self.release/'sensors':'/sensors',self.release/'deps':'/deps',self.root:'/evidence'}
        for src,dst in mounts.items():args+=['--mount',f'type=bind,source={src.as_posix()},target={dst},readonly']
        args+=['--mount',f'type=bind,source={out.as_posix()},target=/result']
        if detached:args+=['-d']
        args+=['--entrypoint','bash',image,'-lc','set -eo pipefail\n'+script]
        (out/'command.json').write_text(json.dumps(args,indent=2))
        if detached:subprocess.run(args,check=True)
        else:
            with (out/'console.log').open('x',encoding='utf-8') as f:code=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT).returncode
            print(role,'exit',code);return code
    def sensors(self):
        self.ground(self.root)
        if self.local.get('diagnostics',{}).get('enabled'):
            self.run('diagnostics',self.images['onboard'],ONBOARD+'source /racer-run/install/local_setup.bash\nexec python3 /multi/group.py diagnostics')
            deadline=time.monotonic()+self.fleet['readiness_timeout_s']
            while not all((self.root/'diagnostics'/v['namespace']/'ready.json').exists() for v in self.fleet['vehicles']):
                if not read_inspect(self.prefix+'-diagnostics')['State']['Running']:raise RuntimeError('diagnostic recorder exited before readiness')
                if time.monotonic()>deadline:raise TimeoutError('diagnostic recorder readiness timeout')
                time.sleep(.2)
            self.run('observer',self.images['flight'],ROS+'exec python3 /single/observe.py')
        for v in self.fleet['vehicles']:
            ns=v['namespace'];self.run('camera-'+ns,self.images['flight'],ROS+
                'exec python3 -c "from uav_as2_sensor_adapter.node import main; main()" --ros-args '+
                '-r __node:=camera_'+ns+' -p settings_file:=/config/'+ns+'/camera-bridge.json -p vehicle_name:='+v['airsim']+
                ' -p vehicle_namespace:='+ns+' -p rpc_host:="$AIRSIM_HOST" -p rpc_port:="$AIRSIM_RPC_PORT" '+
                '-p use_sim_time:=true -p camera_rate_hz:='+str(self.fleet.get('camera_rate_hz',3.0))+' -p camera_reliable:=true -p timing_file:=/result/timing.jsonl')
        self.run('inputs',self.images['onboard'],ONBOARD+'exec python3 /multi/group.py inputs')
        self.run('overhead',self.images['flight'],ROS+'exec python3 /single/overhead.py')
    def start(self):
        self.ground(self.root)
        source={p.relative_to(REPO).as_posix():sha(p) for folder in [DEMO,SINGLE,MULTI,REPO/'src/tool/navigation/planner/racer/ros2_adapter/bridge',REPO/'deployment/swarm'] for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        for rel in source:
            dst=self.root/'source-snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(REPO/rel,dst)
        (self.root/'runtime-source.json').write_text(json.dumps(source,indent=2))
        self.run('native',self.images['algorithm'],ROS+'source /racer-run/install/setup.bash\ncd /deps\nsha256sum -c nlopt-debs.sha256\ndpkg -i nlopt-debs/*.deb\ncd /racer-run\nsha256sum -c executable.sha256\nexec python3 /multi/group.py native')
        self.run('shadow',self.images['onboard'],ONBOARD+'source /racer-run/install/local_setup.bash\nexec python3 /multi/group.py shadow')
    def mission(self):
        self.ground(self.root)
        for rel,h in read(self.root/'runtime-source.json').items():
            if sha(REPO/rel)!=h:raise ValueError('runtime changed '+rel)
        return self.run('mission',self.images['workstation'],ROS+'source /interfaces/install/local_setup.bash\nsource /racer-run/install/local_setup.bash\nexec python3 /code/mission.py',False)
    def stop(self):
        self.ground(self.root)
        states=[]
        for role in ['shadow','native','inputs']+['camera-'+v['namespace'] for v in self.fleet['vehicles']]+['overhead']+(['observer','diagnostics'] if self.local.get('diagnostics',{}).get('enabled') else []):
            name=self.prefix+'-'+role;info=read_inspect(name)
            if info['Config']['Labels'].get('aerohub.racer.owner')!=self.prefix:raise ValueError('owner mismatch')
            if info['State']['Running']:call(['docker','stop','-t','30',name])
            with (self.root/role/'container.log').open('w',encoding='utf-8') as f:subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT)
            item=read_inspect(name);states.append({'name':name,'State':item['State']})
        (self.root/'cleanup-result.json').write_text(json.dumps(states,indent=2))
        if any(s['State']['Running'] or s['State']['ExitCode'] for s in states):raise RuntimeError('cleanup failed; preserve evidence')
    def down(self):
        self.ground(self.root)
        if call(['docker','ps','-q','--filter','label=aerohub.racer.owner='+self.prefix]):raise RuntimeError('owned workers still running')
        subprocess.run(self.compose+['stop','-t','15'],check=True)
        roles=list(read(self.root/'compose.json')['services']);states=[]
        for role in roles:
            name=self.prefix+'-'+role+'-1';info=read_inspect(name)
            states.append({k:info[k] for k in ['Name','Image','State']})
            with (self.root/(role+'-container.log')).open('w',encoding='utf-8') as f:subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT)
        (self.root/'core-final.json').write_text(json.dumps(states,indent=2))
        if any(s['State']['Running'] or s['State']['ExitCode'] for s in states):raise RuntimeError('base service cleanup failed')
def read_inspect(name,timeout=None):return json.loads(call(['docker','inspect',name],timeout=timeout))[0]
if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--local',type=Path,required=True)
    ap.add_argument('command',choices=['up','sensors','start','mission','stop','down']);a=ap.parse_args()
    raise SystemExit(getattr(Deployment(a.local),a.command)() or 0)
