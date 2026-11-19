import json,tempfile,unittest
from pathlib import Path
from full_flow import execute

class FakeDeployment:
    def __init__(self, root, task_code=0, ground_lost=False):
        self.root=root;self.task_code=task_code;self.ground_lost=ground_lost;self.task_done=False;self.cleaned=[]
    def up(self):
        (self.root/'mission').mkdir();(self.root/'mission/result.json').write_text(json.dumps({'pass':not self.task_code}))
    def ground(self, out):
        if self.task_done and self.ground_lost:raise RuntimeError('not landed')
    def sensors(self):pass
    def survey(self):pass
    def start(self):pass
    def logger(self, stop=False):self.cleaned.append('logger-stop' if stop else 'logger-start')
    def mission_mode(self,mode):self.last_mode=mode;self.task_done=True;return self.task_code
    def stop(self):self.cleaned.append('stop')
    def down(self):self.cleaned.append('down')

class FlowTests(unittest.TestCase):
    def run_case(self, code, lost):
        with tempfile.TemporaryDirectory() as folder:
            d=FakeDeployment(Path(folder),code,lost);result=execute(d)
            saved=json.loads((d.root/'flow-result.json').read_text())
            return d,result,saved
    def test_explicit_drone1_mode_is_forwarded(self):
        with tempfile.TemporaryDirectory() as folder:
            d=FakeDeployment(Path(folder));self.assertEqual(execute(d,'takeoff-drone1'),0)
            self.assertEqual(d.last_mode,'takeoff-drone1')
    def test_success_includes_cleanup(self):
        d,code,r=self.run_case(0,False);self.assertEqual(code,0);self.assertTrue(r['pass']);self.assertEqual(d.cleaned[-3:],['logger-stop','stop','down'])
    def test_failed_task_stays_failed_after_safe_cleanup(self):
        d,code,r=self.run_case(1,False);self.assertEqual(code,1);self.assertFalse(r['pass']);self.assertEqual(d.cleaned[-3:],['logger-stop','stop','down'])
    def test_missing_ground_preserves_flight_services(self):
        d,code,r=self.run_case(1,True);self.assertEqual(code,1);self.assertEqual(d.cleaned,['logger-start'])

if __name__=='__main__':unittest.main()
