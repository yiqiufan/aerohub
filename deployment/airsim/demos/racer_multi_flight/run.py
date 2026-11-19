"""Two-UAV real execution entry; existing ground entry remains shadow-only."""
import argparse,hashlib,json,shutil,subprocess,sys
from pathlib import Path
DEMO=Path(__file__).resolve().parent;REPO=DEMO.parents[3]
sys.path.insert(0,str(DEMO.parent/'racer_multi_ground'))
from run import Deployment,ROS,ONBOARD,read_inspect,call,sha

class FlightDeployment(Deployment):
    def __init__(self,path):
        super().__init__(path)
        self.flight_path=self.config_path(path,'flight_config',DEMO/'flight.json')
        if 'flight_config' in self.local:self.local['flight_config']=str(self.flight_path)
        self.flight_config=json.loads(self.flight_path.read_text())
        self.flight_source_sha=sha(self.flight_path)
        if self.flight_config['planning_box_enu']!=self.fleet['planning_box_enu']:
            raise ValueError('fleet/flight planning boxes differ')

    def validate_completion_config(self):
        from completion import validate_budget
        return validate_budget(self.local.get('completion_wall_budget_s'))

    def up(self):
        if len(self.fleet['vehicles']) != 2:
            raise ValueError('This flight coordinator is validated for exactly two UAVs')
        if 'completion_wall_budget_s' in self.local:self.validate_completion_config()
        if sha(self.flight_path)!=self.flight_source_sha:raise ValueError('selected flight configuration changed')
        super().up();(self.root/'configuration/flight.json').write_text(json.dumps(self.flight_config,indent=2))
        provenance=self.root/'configuration-sources.json';origins=json.loads(provenance.read_text())
        origins['flight']={'path':str(self.flight_path),'sha256':self.flight_source_sha}
        provenance.write_text(json.dumps(origins,indent=2))
        if 'completion_wall_budget_s' in self.local:
            budget=self.validate_completion_config()
            path=self.root/'configuration/flight.json';cfg=json.loads(path.read_text())
            cfg['completion_wall_budget_s']=budget;path.write_text(json.dumps(cfg,indent=2))
    def sensors(self):
        super().sensors()
        if not self.local.get('diagnostics',{}).get('enabled'):
            self.run('observer',self.images['flight'],ROS+'exec python3 /single/observe.py')
    def survey(self):
        self.ground(self.root)
        return self.run('survey',self.images['flight'],ROS+'export PYTHONPATH=/single:$PYTHONPATH\nexec python3 /repo/deployment/airsim/demos/racer_multi_flight/survey.py',False)
    def start(self):
        self.ground(self.root)
        source={p.relative_to(REPO).as_posix():sha(p) for folder in [DEMO,DEMO.parent/'racer_multi_ground',DEMO.parent/'racer_single',REPO/'src/tool/navigation/planner/racer/ros2_adapter/multi',REPO/'src/tool/navigation/planner/racer/ros2_adapter/bridge',REPO/'deployment/swarm'] for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        for rel in source:
            dst=self.root/'source-snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(REPO/rel,dst)
        (self.root/'runtime-source.json').write_text(json.dumps(source,indent=2))
        self.run('native',self.images['algorithm'],ROS+'source /racer-run/install/setup.bash\ncd /deps\nsha256sum -c nlopt-debs.sha256\ndpkg -i nlopt-debs/*.deb\ncd /racer-run\nsha256sum -c executable.sha256\nexec python3 /multi/group.py native')
        self.run('adapter',self.images['onboard'],ONBOARD+'source /racer-run/install/local_setup.bash\nexec python3 /multi/group.py flight_adapter')
    def mission_mode(self,mode):
        if mode=='explore-complete':
            from completion import validate_budget
            validate_budget(json.loads((self.root/'configuration/flight.json').read_text()).get('completion_wall_budget_s'))
        self.ground(self.root)
        for rel,h in json.loads((self.root/'runtime-source.json').read_text()).items():
            if sha(REPO/rel)!=h:raise ValueError('runtime source changed '+rel)
        return self.run('mission',self.images['workstation'],ROS+'source /interfaces/install/local_setup.bash\nsource /racer-run/install/local_setup.bash\nexport PYTHONPATH=/single:$PYTHONPATH\nexec python3 /repo/deployment/airsim/demos/racer_multi_flight/mission.py '+mode,False)
    def stop(self):
        self.ground(self.root);states=[]
        for role in ['adapter','native','inputs']+['camera-'+v['namespace'] for v in self.fleet['vehicles']]+['overhead','observer']+(['diagnostics'] if self.local.get('diagnostics',{}).get('enabled') else []):
            name=self.prefix+'-'+role;info=read_inspect(name)
            if info['Config']['Labels'].get('aerohub.racer.owner')!=self.prefix:raise ValueError('owner mismatch')
            if info['State']['Running']:call(['docker','stop','-t','30',name])
            with (self.root/role/'container.log').open('w',encoding='utf-8') as f:subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT)
            states.append({'name':name,'State':read_inspect(name)['State']})
        (self.root/'cleanup-result.json').write_text(json.dumps(states,indent=2))
        if any(s['State']['Running'] or s['State']['ExitCode'] for s in states):raise RuntimeError('cleanup failed')
    def logger(self,stop=False):
        self.ground(self.root);out=self.root/'ulog';out.mkdir(exist_ok=True)
        for v in self.fleet['vehicles']:
            ns=v['namespace'];container=self.core(ns);args=['docker','exec',container,'/opt/px4-gazebo/bin/px4-logger','--instance',str(v['px4_instance'])]
            if not stop:
                before=subprocess.run(args+['status'],text=True,capture_output=True)
                (out/(ns+'-before.log')).write_text(before.stdout+before.stderr)
                if before.returncode==0:call(args+['stop'])
                call(args+['start','-m','file','-b','256','-t','-e','-f'])
            status=call(args+['status']);(out/(ns+('-final.log' if stop else '-start.log'))).write_text(status)
            if stop:
                import re
                match=re.search(r'Log file:\s*(\S+\.ulg)',status)
                if not match:raise RuntimeError('ULog missing '+ns)
                call(args+['stop']);subprocess.run(['docker','cp',container+':/var/lib/flight_board/'+match.group(1).lstrip('./'),str(out/(ns+'.ulg'))],check=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--local',type=Path,required=True)
    ap.add_argument('command',choices=['up','sensors','survey','start','takeoff','takeoff-drone1','explore','explore-complete','stop','down','logger-start','logger-stop'])
    a=ap.parse_args();d=FlightDeployment(a.local)
    if a.command in ['takeoff','takeoff-drone1','explore','explore-complete']:code=d.mission_mode(a.command)
    elif a.command.startswith('logger-'):code=d.logger(a.command=='logger-stop')
    else:code=getattr(d,a.command)()
    raise SystemExit(code or 0)
