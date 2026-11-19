"""PC single-RACER deployment. Local paths/network belong in --local, not source."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

DEMO=Path(__file__).resolve().parent
REPO=DEMO.parents[3]
ROS='source /opt/ros/humble/setup.bash\n'
ONBOARD=ROS+'source /root/aerostack2_ws/install/setup.bash\nsource /opt/px4_ws/install/setup.bash\nsource /opt/uav_execution_ws/install/setup.bash\n'
NATIVE=ROS+'source /racer-run/install/setup.bash\n'

def call(args):
    return subprocess.check_output(args,text=True,encoding='utf-8',errors='replace').strip()

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

class Deployment:
    def __init__(self,local):
        self.local=json.loads(local.read_text(encoding='utf-8'))
        self.root=Path(self.local['run_root']).resolve()
        self.release=Path(self.local['release']).resolve()
        self.prefix=self.local['project']
        if not re.fullmatch('[a-z][a-z0-9-]{2,48}',self.prefix):raise ValueError('invalid project name')
        self.images=json.loads((DEMO/'images.lock.json').read_text())
        self.compose=['docker','compose','-p',self.prefix,'-f',str(self.root/'compose.json')]

    def core(self,role):
        return call(self.compose+['ps','-q',role])

    def ground(self,output):
        evidence={}
        for i in (0,1):
            container=self.core('drone'+str(i))
            if not container:raise RuntimeError('PX4 container unavailable')
            cmd=['docker','exec',container,'/opt/px4-gazebo/bin/px4-listener','--instance',str(i)]
            status=call(cmd+['vehicle_status','-n','1'])
            land=call(cmd+['vehicle_land_detected','-n','1'])
            evidence[str(i)]={'status':status,'land':land}
        (output/'px4-ground.json').write_text(json.dumps(evidence,indent=2))
        if not all(re.search(r'arming_state: 1\b',v['status']) and re.search(r'\blanded: True\b',v['land']) for v in evidence.values()):
            raise RuntimeError('both PX4 instances must confirm landed/disarmed; resources preserved')

    def run(self,role,image,script,output,detached=True):
        name=self.prefix+'-'+role
        mounts={REPO:'/repo',DEMO:'/code',REPO/'src/tool/navigation/planner/racer/ros2_adapter':'/component',
                self.root/'configuration':'/config',self.release/'native':'/racer-run',
                self.release/'interfaces':'/interfaces',self.release/'sensors':'/sensors',self.release/'deps':'/deps'}
        args=['docker','run','--init','--pull=never','--no-healthcheck','--name',name,
              '--label','aerohub.racer.owner='+self.prefix,'--network',self.prefix+'_default',
              '-e','ROS_DOMAIN_ID='+str(self.local['domain']),'-e','AIRSIM_HOST='+self.local['airsim_host'],
              '-e','AIRSIM_RPC_PORT='+str(self.local.get('rpc_port',41461)),
              '-e','PYTHONDONTWRITEBYTECODE=1','-e','ROS_LOG_DIR=/result/ros-logs',
              '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds_udp.xml',
              '-e','PYTHONPATH=/component/bridge:/sensors:/repo:/code']
        for source,target in mounts.items():args+=['--mount',f'type=bind,source={source.as_posix()},target={target},readonly']
        args+=['--mount',f'type=bind,source={output.as_posix()},target=/result']
        if detached:args+=['-d']
        args+=['--entrypoint','bash',image,'-lc','set -eo pipefail\n'+script]
        (output/(role+'-command.json')).write_text(json.dumps(args,indent=2))
        if detached:subprocess.run(args,check=True)
        else:
            with (output/(role+'.log')).open('x',encoding='utf-8') as f:r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT)
            print(role,'exit',r.returncode,flush=True)
            return r.returncode

    def stop_one(self,role,output):
        name=self.prefix+'-'+role
        info=json.loads(call(['docker','inspect',name]))[0]
        if info['Config']['Labels'].get('aerohub.racer.owner')!=self.prefix:raise RuntimeError('ownership mismatch')
        if info['State']['Running']:call(['docker','stop','-t','15',name])
        with (output/(role+'-final.log')).open('w',encoding='utf-8') as f:
            subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT,check=True)
        state=json.loads(call(['docker','inspect',name]))[0]['State']
        (output/(role+'-exit.json')).write_text(json.dumps(state,indent=2))
        return not state['Running'] and state['ExitCode']==0

    def up(self):
        self.root.mkdir(parents=True,exist_ok=False)
        manifest=json.loads((self.release/'manifest.json').read_text(encoding='utf-8'))
        if manifest['status']!='built':raise RuntimeError('candidate not built')
        if manifest['images']!=self.images:raise RuntimeError('runtime and build image locks differ')
        lock=json.loads((DEMO/'dependencies.lock.json').read_text())
        for rel,expected in lock['files'].items():
            if sha(self.release/'deps'/rel)!=expected:raise RuntimeError('dependency differs: '+rel)
        for rel,expected in manifest['source_files'].items():
            if '/native/' in rel or '/workstation_interfaces/' in rel:
                if sha(REPO/rel)!=expected:raise RuntimeError('source differs from build: '+rel)
        for rel,expected in manifest['artifacts'].items():
            if sha(self.release/rel)!=expected:raise RuntimeError('artifact differs: '+rel)
        shutil.copytree(DEMO/'config',self.root/'configuration')
        shutil.copyfile(self.local['settings'],self.root/'settings.json')
        shutil.copyfile(self.release/'manifest.json',self.root/'build-manifest.json')
        (self.root/'local.json').write_text(json.dumps(self.local,indent=2))
        services={}
        common={'AIRSIM_HOST':self.local['airsim_host'],'AIRSIM_RPC_PORT':str(self.local.get('rpc_port',41461)),
                'ROS_DOMAIN_ID':str(self.local['domain']),'FLIGHT_BOARD_SENSORS':'0','FLIGHT_BOARD_CAMERAS':'0'}
        def volume(source,target):return {'type':'bind','source':str(source),'target':target,'read_only':True}
        for i in (0,1):
            services['drone'+str(i)]={'image':self.images['flight'],'pull_policy':'never','init':True,'command':['vehicle'],
                'environment':dict(common,VEHICLE_NAME='Drone'+str(i),VEHICLE_NAMESPACE='drone'+str(i),
                    PX4_INSTANCE=str(i),PX4_SYSTEM_ID=str(135+i),XRCE_AGENT_PORT=str(8888+i)),
                'ports':[f'127.0.0.1:{14583+i}:{14583+i}/udp'],
                'volumes':[volume(self.root/'settings.json','/config/settings.json')]}
        services['clock']={'image':self.images['flight'],'pull_policy':'never','init':True,'command':['clock'],
            'environment':dict(common,VEHICLE_NAME='Drone0')}
        services['as2']={'image':self.images['onboard'],'pull_policy':'never','init':True,
            'environment':{'ROS_DOMAIN_ID':str(self.local['domain']),'AS2_FLEET_FILE':'/config/fleet.yaml','AS2_EXECUTION_BACKEND':'follow_path'},
            'volumes':[volume(self.root/'configuration/fleet.yaml','/config/fleet.yaml')]}
        services['navigation']={'image':self.images['onboard'],'pull_policy':'never','init':True,
            'environment':{'ROS_DOMAIN_ID':str(self.local['domain'])},
            'volumes':[volume(self.root/'configuration/navigation.yaml','/config/navigation.yaml')],
            'stop_signal':'SIGINT','command':['/root/aerostack2_ws/install/lib/as2_navigation_inputs/navigation_inputs_node','--ros-args','-r','__ns:=/drone0','--params-file','/config/navigation.yaml']}
        (self.root/'compose.json').write_text(json.dumps({'services':services},indent=2))
        subprocess.run(self.compose+['up','-d'],check=True)

    def sensors(self):
        out=self.root/'sensors';out.mkdir(exist_ok=False)
        self.run('observer',self.images['flight'],ROS+'exec python3 /code/observe.py',out)
        self.run('camera',self.images['flight'],ROS+'exec python3 -c "from uav_as2_sensor_adapter.node import main; main()" --ros-args -p settings_file:=/config/camera-bridge.json -p vehicle_name:=Drone0 -p vehicle_namespace:=drone0 -p rpc_host:="$AIRSIM_HOST" -p rpc_port:="$AIRSIM_RPC_PORT" -p use_sim_time:=true -p camera_rate_hz:=3.0 -p camera_reliable:=true -p timing_file:=/result/camera-timing.jsonl',out)
        self.run('inputs',self.images['onboard'],ONBOARD+'exec python3 /component/bridge/inputs.py --ros-args -p use_sim_time:=true',out)
        self.run('overhead',self.images['flight'],ROS+'exec python3 /code/overhead.py',out)

    def case(self,name,fly):
        if not re.fullmatch('[a-z0-9-]+',name or ''):raise ValueError('case required')
        out=self.root/name;out.mkdir(exist_ok=False)
        self.ground(out)
        self.run('survey-'+name,self.images['flight'],ROS+'exec python3 /code/survey.py',out,False)
        source={p.relative_to(REPO).as_posix():sha(p) for folder in [DEMO,REPO/'src/tool/navigation/planner/racer/ros2_adapter/bridge',REPO/'deployment/swarm']
                for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        for rel in source:
            target=out/'source-snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(REPO/rel,target)
        (out/'runtime-source.json').write_text(json.dumps(source,indent=2))
        self.run('native-'+name,self.images['algorithm'],NATIVE+'cd /deps\nsha256sum -c nlopt-debs.sha256\ndpkg -i nlopt-debs/*.deb\ncd /racer-run\nsha256sum -c executable.sha256\nexec python3 /component/bridge/native.py',out)
        self.run('adapter-'+name,self.images['onboard'],ONBOARD+'source /racer-run/install/local_setup.bash\nexec python3 /component/bridge/adapter_service.py'+('' if fly else ' --dry-run'),out)

    def mission(self,name,fly):
        out=self.root/name
        for rel,expected in json.loads((out/'runtime-source.json').read_text()).items():
            if sha(REPO/rel)!=expected:raise RuntimeError('runtime changed after case start: '+rel)
        result=self.run('mission-'+name,self.images['workstation'],ROS+'source /interfaces/install/local_setup.bash\nexec python3 /code/mission.py'+(' --fly' if fly else ''),out,False)
        return result

    def stop(self,name):
        out=self.root/name
        self.ground(out)
        results={role:self.stop_one(role+'-'+name,out) for role in ('adapter','native')}
        (out/'cleanup-result.json').write_text(json.dumps(results,indent=2))
        if not all(results.values()):raise RuntimeError('component cleanup failed; inspect raw logs')

    def stop_sensors(self):
        out=self.root/'sensors';self.ground(out)
        results={role:self.stop_one(role,out) for role in ('inputs','camera','observer','overhead')}
        (out/'cleanup-result.json').write_text(json.dumps(results,indent=2))
        if not all(results.values()):raise RuntimeError('sensor cleanup failed; inspect raw logs')

    def logger(self,stop=False,case=None):
        out=self.root/case;container=self.core('drone0')
        args=['docker','exec',container,'/opt/px4-gazebo/bin/px4-logger','--instance','0']
        if not stop:
            self.ground(out)
            before=subprocess.run(args+['status'],text=True,capture_output=True)
            (out/'logger-before.log').write_text(before.stdout+before.stderr)
            if before.returncode==0:
                # The owned SITL can boot an idle logger; replace it only on ground.
                call(args+['stop'])
            call(args+['start','-m','file','-b','256','-t','-e','-f'])
        status=call(args+['status']);(out/('logger-final.log' if stop else 'logger-start.log')).write_text(status)
        if stop:
            self.ground(out)
            match=re.search(r'Log file:\s*(\S+\.ulg)',status)
            if not match:raise RuntimeError('active log file missing')
            call(args+['stop'])
            path=match.group(1).lstrip('./')
            subprocess.run(['docker','cp',container+':/var/lib/flight_board/'+path,str(out/'flight.ulg')],check=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--local',type=Path,required=True)
    ap.add_argument('command',choices=['up','sensors','start','mission','stop','stop-sensors','down','logger-start','logger-stop','cancel'])
    ap.add_argument('--case');ap.add_argument('--fly',action='store_true')
    a=ap.parse_args();d=Deployment(a.local)
    if a.command=='up':d.up()
    elif a.command=='sensors':d.sensors()
    elif a.command=='start':d.case(a.case,a.fly)
    elif a.command=='mission':return d.mission(a.case,a.fly)
    elif a.command=='stop':d.stop(a.case)
    elif a.command=='stop-sensors':d.stop_sensors()
    elif a.command.startswith('logger-'):d.logger(a.command=='logger-stop',a.case)
    elif a.command=='cancel':call(['docker','kill','--signal','SIGINT',d.prefix+'-mission-'+a.case])
    elif a.command=='down':
        d.ground(d.root)
        active=call(['docker','ps','-q','--filter','label=aerohub.racer.owner='+d.prefix])
        if active:raise RuntimeError('stop owned algorithm/sensor/task containers first')
        subprocess.run(d.compose+['stop','-t','15'],check=True)
        names=[d.prefix+'-'+role+'-1' for role in ('drone0','drone1','clock','as2','navigation')]
        states=json.loads(call(['docker','inspect',*names]))
        rows=[{k:row[k] for k in ('Name','Image','State')} for row in states]
        (d.root/'core-final.json').write_text(json.dumps(rows,indent=2))
        if any(row['State']['Running'] or row['State']['ExitCode']!=0 for row in rows):
            raise RuntimeError('base-service cleanup did not finish normally; inspect core-final.json')
    return 0

if __name__=='__main__':raise SystemExit(main())
