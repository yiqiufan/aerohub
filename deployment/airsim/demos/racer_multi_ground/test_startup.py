"""Startup ordering and failure gates; no Docker or simulated motion."""
import importlib.util,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch,Mock
spec=importlib.util.spec_from_file_location('racer_startup',Path(__file__).with_name('run.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class StartupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.d=m.Deployment.__new__(m.Deployment);self.d.root=Path(self.tmp.name);self.d.prefix='test-racer'
        self.d.compose=['docker','compose','-p','test-racer']
        self.d.fleet={'vehicles':[{'namespace':'drone0','px4_instance':0},{'namespace':'drone1','px4_instance':1}]}
        self.d.ground=Mock();self.commands=[]
        self.enter(patch.object(m.subprocess,'run',side_effect=lambda a,**kw:self.commands.append(a)))
        self.inspect=self.enter(patch.object(m,'read_inspect',side_effect=lambda n,**kw:{'Name':n,'Image':'fixed','State':{'Running':True,'ExitCode':0}}))
        self.dds=self.enter(patch.object(m,'call',return_value='Running, connected'))
        self.enter(patch.object(m.time,'sleep'))
    def enter(self, patcher):
        self.addCleanup(patcher.stop)
        return patcher.start()
    def ups(self):return [x for x in self.commands if 'up' in x]
    def report(self):return json.loads((self.d.root/'startup-result.json').read_text())
    def test_compute_waits_for_ground_and_both_dds(self):
        self.d.ground.side_effect=[RuntimeError('not landed'),None,None]
        self.dds.side_effect=['Running, connected','disconnected','Running, connected','Running, connected']
        self.d.start_core();self.assertTrue(self.report()['pass']);self.assertEqual(self.d.ground.call_count,3)
        self.assertEqual(self.ups(),[self.d.compose+['up','-d','clock','drone0','drone1'],self.d.compose+['up','-d','as2-drone0','as2-drone1','navigation-drone0','navigation-drone1']])
    def test_crash_retains_evidence_without_compute_or_restart(self):
        self.inspect.side_effect=lambda n,**kw:{'Name':n,'Image':'fixed','State':{'Running':not n.endswith('-drone1-1'),'ExitCode':245 if n.endswith('-drone1-1') else 0}}
        with self.assertRaisesRegex(RuntimeError,'drone1 exited'):self.d.start_core()
        self.assertEqual(len(self.ups()),1);self.assertFalse(self.report()['pass'])
        self.assertTrue((self.d.root/'startup-failure-containers.json').exists())
        self.assertFalse(any('stop' in x or 'restart' in x for x in self.commands))
    def test_ground_timeout_blocks_compute(self):
        self.d.ground.side_effect=RuntimeError('armed')
        with self.assertRaisesRegex(RuntimeError,'readiness timeout'):self.d.start_core(timeout=0)
        self.assertEqual(len(self.ups()),1);self.dds.assert_not_called()
    def test_disconnected_dds_blocks_compute(self):
        self.dds.return_value='disconnected'
        with self.assertRaisesRegex(RuntimeError,'DDS not connected'):self.d.start_core(timeout=0)
        self.assertEqual(len(self.ups()),1)
    def test_second_phase_failure_is_not_success(self):
        def run(args,**kw):
            self.commands.append(args)
            if 'as2-drone0' in args:raise m.subprocess.CalledProcessError(1,args)
        with patch.object(m.subprocess,'run',side_effect=run):
            with self.assertRaises(m.subprocess.CalledProcessError):self.d.start_core()
        self.assertFalse(self.report()['pass'])
    def test_roles_follow_fleet_instead_of_hardcoded_pair(self):
        self.d.fleet['vehicles'].append({'namespace':'drone2','px4_instance':2})
        self.d.start_core();self.assertIn('drone2',self.ups()[0]);self.assertIn('as2-drone2',self.ups()[1])
if __name__=='__main__':unittest.main()
