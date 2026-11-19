from pathlib import Path
import argparse,json,subprocess,hashlib
p=argparse.ArgumentParser();p.add_argument('--release',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--params',type=Path,required=True);p.add_argument('--baseline',action='store_true');a=p.parse_args()
m=json.loads((a.release/'manifest.json').read_text());assert m['status']=='built'
def sha(path):
 with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for rel,digest in m['artifacts'].items():assert sha(a.release/rel)==digest,rel
a.output.mkdir(parents=True,exist_ok=False)
(a.output/'inputs.json').write_text(json.dumps({'release_manifest_sha256':sha(a.release/'manifest.json'),'params_sha256':sha(a.params),'test_files':{f.name:sha(f) for f in (Path(__file__).parent/'test').iterdir() if f.is_file()},'artifact_hashes_verified':True},indent=2))
script='''set -eo pipefail
source /opt/ros/humble/setup.bash
dpkg -i /candidate/deps/nlopt-debs/*.deb
cp -a /candidate/native/install /tmp/probe-install
source /tmp/probe-install/setup.bash
cmake -S /test -B /tmp/probe-build -DWITH_DIAGNOSTICS=MODE
cmake --build /tmp/probe-build -j1
'''.replace('MODE','OFF' if a.baseline else 'ON')
for mode in (['baseline'] if a.baseline else ['off','on']):
 script+=f'PROBE_RESULT=/result/{mode}-views.json RACER_DIAGNOSTIC_FILE=/result/{mode}-map.jsonl /tmp/probe-build/frontier_probe --ros-args --params-file /params/native-params.yaml -p use_sim_time:=false -p frontier.decision_diagnostics:={str(mode=="on").lower()} > /result/{mode}.log 2>&1\n'
(a.output/'run.sh').write_text(script,newline='\n')
cmd=['docker','run','--rm','--init','--network','none','--pull=never','--name','racer-frontier-probe-'+('baseline' if a.baseline else 'candidate'),
 '--mount',f'type=bind,source={a.release.as_posix()},target=/candidate,readonly',
 '--mount',f'type=bind,source={(Path(__file__).parent/"test").as_posix()},target=/test,readonly',
 '--mount',f'type=bind,source={a.output.as_posix()},target=/result',
 '--mount',f'type=bind,source={a.params.as_posix()},target=/params/native-params.yaml,readonly',
 '--entrypoint','bash',m['images']['algorithm'],'/result/run.sh']
(a.output/'command.json').write_text(json.dumps(cmd,indent=2))
try:
 with (a.output/'build.log').open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=900)
except subprocess.TimeoutExpired:
 cleanup=subprocess.run(['docker','stop','-t','10','racer-frontier-probe-'+('baseline' if a.baseline else 'candidate')],capture_output=True,text=True,timeout=30)
 (a.output/'exit.json').write_text(json.dumps({'exit':None,'timeout':True,'owned_container_stop_exit':cleanup.returncode,'cleanup':cleanup.stdout+cleanup.stderr}))
 raise
(a.output/'exit.json').write_text(json.dumps({'exit':r.returncode}))
print(a.output,r.returncode);raise SystemExit(r.returncode)
