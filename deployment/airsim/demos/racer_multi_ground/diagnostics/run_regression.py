"""Run the real native binary with synthetic inputs in a network-isolated container."""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path

DEMO=Path(__file__).resolve().parents[1];REPO=DEMO.parents[3]
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--release',type=Path,required=True)
    ap.add_argument('--configuration',type=Path,required=True,help='configuration directory from a recorded ground case')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--mode',choices=['busy-stop','busy-shutdown','accept-stop'],default='busy-stop')
    a=ap.parse_args();release=a.release.resolve();out=a.output.resolve();config=a.configuration.resolve()
    manifest=json.loads((release/'manifest.json').read_text())
    if manifest['status']!='built':raise ValueError('release not built')
    for rel,h in manifest['artifacts'].items():
        if sha(release/rel)!=h:raise ValueError('release artifact mismatch '+rel)
    out.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(release/'manifest.json',out/'build-manifest.json')
    shutil.copytree(config,out/'configuration-snapshot')
    multi=REPO/'src/tool/navigation/planner/racer/ros2_adapter/multi'
    for p in [multi/'native.py',multi/'common.py',DEMO/'diagnostics/stop_regression.py',Path(__file__)]:shutil.copyfile(p,out/('source-'+p.name))
    image=manifest['images']['algorithm']
    args=['docker','run','--rm','--init','--pull=never','--network','none','-e','ROS_DOMAIN_ID=98',
          '-e','PYTHONDONTWRITEBYTECODE=1','-e','PYTHONPATH=/multi:/component/bridge:/repo']
    mounts=[(REPO,'/repo',True),(REPO/'src/tool/navigation/planner/racer/ros2_adapter','/component',True),(multi,'/multi',True),
            (out/'configuration-snapshot','/config',True),(release/'native','/racer-run',True),(release/'deps','/deps',True),
            (out/'source-stop_regression.py','/probe.py',True),(out,'/result',False)]
    for src,dst,ro in mounts:args+=['--mount',f'type=bind,source={src.as_posix()},target={dst}'+(',readonly' if ro else '')]
    script='set -e\nsource /opt/ros/humble/setup.bash\nsource /racer-run/install/setup.bash\ncd /deps\nsha256sum -c nlopt-debs.sha256\ndpkg -i nlopt-debs/*.deb\nexec python3 /probe.py --mode '+a.mode
    args+=['--entrypoint','bash',image,'-lc',script]
    (out/'command.json').write_text(json.dumps(args,indent=2))
    with (out/'console.log').open('x',encoding='utf-8') as log:result=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT)
    report=out/'result.json'
    print(report.read_text() if report.exists() else 'Result missing; inspect '+str(out/'console.log'))
    return result.returncode
if __name__=='__main__':raise SystemExit(main())
