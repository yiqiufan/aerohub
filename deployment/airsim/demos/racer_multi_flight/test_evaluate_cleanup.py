import importlib.util,json,tempfile,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('candidate',HERE/'evaluate.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

class CleanupEvidence(unittest.TestCase):
    def fixture(self,root,diagnostics):
        fleet={'vehicles':[{'namespace':'drone0'},{'namespace':'drone1'}]}
        roles=['adapter','native','inputs','overhead','observer','camera-drone0','camera-drone1']+(['diagnostics'] if diagnostics else [])
        services=['clock','drone0','drone1','as2-drone0','as2-drone1','navigation-drone0','navigation-drone1']
        files={'local.json':{'project':'case','diagnostics':{'enabled':diagnostics}},
          'compose.json':{'services':dict.fromkeys(services,{})},
          'cleanup-result.json':[{'name':'case-'+s,'State':{'Running':False,'ExitCode':0}} for s in roles],
          'core-final.json':[{'Name':'/case-'+s+'-1','State':{'Running':False,'ExitCode':0}} for s in services]}
        self.save(root,files);return fleet,files
    def save(self,root,files):
        for name,data in files.items():(root/name).write_text(json.dumps(data))
    def test_old_and_diagnostic_layouts(self):
        for enabled in [False,True]:
            with self.subTest(enabled=enabled),tempfile.TemporaryDirectory() as d:
                root=Path(d);fleet,_=self.fixture(root,enabled)
                self.assertTrue(mod.cleanup_checks(root,fleet)['pass'])
    def test_missing_recorder_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);fleet,f=self.fixture(root,True);f['cleanup-result.json'].pop();self.save(root,f)
            self.assertFalse(mod.cleanup_checks(root,fleet)['pass'])
    def test_same_count_wrong_or_duplicate_identity_rejected(self):
        for name in ['foreign-recorder','case-adapter']:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as d:
                root=Path(d);fleet,f=self.fixture(root,True);f['cleanup-result.json'][-1]['name']=name;self.save(root,f)
                self.assertFalse(mod.cleanup_checks(root,fleet)['pass'])
    def test_running_or_failed_core_rejected(self):
        for state in [{'Running':True,'ExitCode':0},{'Running':False,'ExitCode':1}]:
            with self.subTest(state=state),tempfile.TemporaryDirectory() as d:
                root=Path(d);fleet,f=self.fixture(root,True);f['core-final.json'][0]['State']=state;self.save(root,f)
                self.assertFalse(mod.cleanup_checks(root,fleet)['pass'])

if __name__=='__main__':unittest.main()
