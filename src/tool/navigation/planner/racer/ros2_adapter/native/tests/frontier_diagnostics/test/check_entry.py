"""Execute the actual supervisor's config prefix; stop before rclpy.init.
Requires source bound at /repo, native adapter at /component, writable /result.
No substitute algorithm, no solvers/nodes or flight commands are started.
"""
from pathlib import Path
import json,os,shutil,sys,tempfile,types,yaml
root=Path('/repo');adapter=root/'src/planner/racer/ros2_adapter'
code=(adapter/'multi/native.py').read_text().split('rclpy.init(',1)[0]
fleet=json.loads((root/'deployment/airsim/demos/racer_multi_ground/fleet.json').read_text())
results=[]
for label,config,want in [('missing',None,False),('enabled_only',{'enabled':True},False),('disabled',{'enabled':False,'frontier_decisions':True},False),('off',{'enabled':True,'frontier_decisions':False},False),('on',{'enabled':True,'frontier_decisions':True},True)]:
 with tempfile.TemporaryDirectory() as d:
  d=Path(d);cfg=d/'config';cfg.mkdir();out=d/'out';out.mkdir()
  shutil.copyfile(root/'deployment/airsim/demos/racer_single/config/mapper.yaml',cfg/'mapper.yaml')
  if config is not None:(d/'diagnostics.json').write_text(json.dumps(config))
  common=types.ModuleType('common')
  for k,v in dict(FLEET=fleet,VEHICLE=fleet['vehicles'][0],NS='drone0',PREFIX='/racer/drone0',CONFIG=cfg,OUT=out).items():setattr(common,k,v)
  sys.modules['common']=common;os.environ.pop('RACER_DIAGNOSTIC_FILE',None)
  exec(compile(code,str(adapter/'multi/native.py'),'exec'),{})
  params=yaml.safe_load((out/'native-params.yaml').read_text())['/**']['ros__parameters']
  assert params['frontier.decision_diagnostics'] is want
  assert 'diagnostic.probe_points' not in params
  assert bool(os.environ.get('RACER_DIAGNOSTIC_FILE'))==bool(config and config.get('enabled'))
  results.append({'case':label,'frontier_decisions':want,'pass':True})
Path('/result/entry-result.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results))
