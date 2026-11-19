"""One existing Swarm ServiceThread and independent process per PC UAV."""
import json,os,signal,subprocess,sys,threading,time
from pathlib import Path
from deployment.swarm.orchestrator import ServiceThread

root=Path('/result');fleet=json.loads(Path('/config/fleet.json').read_text())
kind=sys.argv[1];stop=threading.Event();services=[]
class Worker(ServiceThread):
    def _terminate(self):
        if self.process is None or self.process.poll() is not None:return
        os.killpg(self.process.pid,signal.SIGINT)
        try:self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.error='worker needed termination escalation'
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=3)
for v in fleet['vehicles']:
    out=root/v['namespace'];out.mkdir(exist_ok=False)
    env=dict(os.environ,RACER_VEHICLE_INDEX=str(v['index']),RACER_OUTPUT=str(out))
    worker=Worker(identity=v['namespace'],placement='pc',command=['python3','/multi/'+kind+'.py'],
                  environment=env,log_path=out/'worker.log',stop_event=stop)
    worker.start();services.append(worker)
signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
for service in services:service.ready.wait(10)
(root/'worker-topology.json').write_text(json.dumps([s.record() for s in services],indent=2))
while not stop.wait(.1):
    if any(s.error or s.exited_before_stop for s in services):stop.set()
for service in services:service.join(25)
records=[s.record() for s in services]
(root/'worker-final.json').write_text(json.dumps(records,indent=2))
raise SystemExit(0 if all(r['returncode']==0 and not r['error'] and not r['exited_before_stop'] for r in records) else 1)
