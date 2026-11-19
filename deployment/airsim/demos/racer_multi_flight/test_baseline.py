import hashlib,importlib.util,json,tempfile,unittest
from pathlib import Path

path=Path(__file__).with_name('check_baseline.py')
spec=importlib.util.spec_from_file_location('identity',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

class IdentityTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
  self.repo=self.root/'repo';self.release=self.root/'release';self.repo.mkdir();self.release.mkdir()
  (self.repo/'native.cpp').write_text('reviewed source');(self.release/'node').write_bytes(b'built node')
  (self.release/'executable.sha256').write_text('checksum list');self.settings=self.root/'settings.json';self.settings.write_text('{}')
  self.manifest={'status':'built','images':{'onboard':'sha256:fixed'},'artifacts':{'node':sha(self.release/'node')}}
  (self.release/'manifest.json').write_text(json.dumps(self.manifest))
  self.lock={'release_manifest_sha256':sha(self.release/'manifest.json'),'images':self.manifest['images'],
    'source_files':{'native.cpp':sha(self.repo/'native.cpp')},'extra_release_files':{'executable.sha256':sha(self.release/'executable.sha256')},
    'settings_sha256':sha(self.settings),'diagnostics':{'enabled':True}}
  self.local={'run_root':str(self.root/'new-run'),'diagnostics':{'enabled':True}}
 def errors(self):return module.check(self.repo,self.release,self.settings,self.lock,self.local)
 def test_matching_files_pass_without_creating_run_directory(self):
  self.assertEqual(self.errors(),[]);self.assertFalse(Path(self.local['run_root']).exists())
 def test_changed_source_rejected(self):
  (self.repo/'native.cpp').write_text('changed');self.assertIn('source: native.cpp',self.errors())
 def test_changed_artifact_rejected(self):
  (self.release/'node').write_bytes(b'wrong node');self.assertIn('artifact: node',self.errors())
 def test_manifest_replacement_rejected(self):
  (self.release/'manifest.json').write_text('{}');self.assertEqual(self.errors(),['release manifest'])
 def test_settings_change_rejected(self):
  self.settings.write_text('{"different":true}');self.assertTrue(any(x.startswith('settings') for x in self.errors()))
 def test_diagnostics_disabled_rejected(self):
  self.local['diagnostics']={'enabled':False};self.assertIn('diagnostics differs from validated case',self.errors())
 def test_existing_evidence_rejected(self):
  Path(self.local['run_root']).mkdir();self.assertIn('run_root already exists; preserve previous evidence',self.errors())
 def test_missing_support_file_rejected(self):
  (self.release/'executable.sha256').unlink();self.assertIn('release support: executable.sha256',self.errors())

if __name__=='__main__':unittest.main(verbosity=2)
