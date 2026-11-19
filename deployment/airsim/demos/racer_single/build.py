"""Build a new, immutable candidate with fixed images and an explicit dependency cache."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import uuid

DEMO=Path(__file__).resolve().parent
REPO=DEMO.parents[3]

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--release',type=Path,required=True)
    ap.add_argument('--deps',type=Path,required=True,help='directory with nlopt-debs and nlopt-debs.sha256')
    args=ap.parse_args()
    release=args.release.resolve();release.mkdir(parents=True,exist_ok=False)
    images=json.loads((DEMO/'images.lock.json').read_text(encoding='utf-8'))
    manifest={'status':'building','images':images,'source_files':{},'commands':[]}
    for folder in [REPO/'src/planner/racer',DEMO,REPO/'simulator/airsim/sensors',REPO/'deployment/swarm']:
        for p in sorted(folder.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                manifest['source_files'][p.relative_to(REPO).as_posix()]=digest(p)
    for image in set(images.values()):
        actual=subprocess.check_output(['docker','image','inspect',image,'--format','{{.Id}}'],text=True).strip()
        if actual!=image:raise RuntimeError('image identity mismatch')
    shutil.copytree(args.deps/'nlopt-debs',release/'deps/nlopt-debs')
    shutil.copyfile(args.deps/'nlopt-debs.sha256',release/'deps/nlopt-debs.sha256')
    dependency_lock=json.loads((DEMO/'dependencies.lock.json').read_text())
    for rel,expected in dependency_lock['files'].items():
        if digest(release/'deps'/rel)!=expected:raise RuntimeError('dependency differs: '+rel)
    manifest['dependency_lock']=dependency_lock
    shutil.copytree(REPO/'simulator/airsim/sensors/uav_as2_sensor_adapter',release/'sensors/uav_as2_sensor_adapter',ignore=shutil.ignore_patterns('__pycache__'))
    lock=json.loads((DEMO/'patches/sensor-lock.json').read_text())
    for name,value in lock.items():
        if digest(release/'sensors/uav_as2_sensor_adapter'/name)!=value['base']:
            raise RuntimeError('sensor patch baseline differs: '+name)
    subprocess.run(['git','apply',str(DEMO/'patches/sensor-compat.patch')],check=True,cwd=release/'sensors')
    # Git apply performs newline conversion on some Windows setups; compare normalized text.
    manifest['sensor_files']={p.name:digest(p) for p in (release/'sensors/uav_as2_sensor_adapter').glob('*.py')}
    for name,value in lock.items():
        if manifest['sensor_files'][name]!=value['candidate']:
            raise RuntimeError('sensor candidate differs from reviewed bytes: '+name)
    for part,image,script,target in [('native',images['algorithm'],'build_native.sh','/racer-run'),
                                    ('interfaces',images['workstation'],'build_interfaces.sh','/interfaces')]:
        out=release/part;out.mkdir()
        cmd=['docker','run','--rm','--init','--network','none','--pull=never','--name','racer-build-'+uuid.uuid4().hex[:10],
             '--mount',f'type=bind,source={REPO.as_posix()},target=/repo,readonly',
             '--mount',f'type=bind,source={out.as_posix()},target={target}',
             '--mount',f'type=bind,source={(release/"deps").as_posix()},target=/deps,readonly',
             '--entrypoint','bash',image,'/repo/deployment/airsim/demos/racer_single/'+script]
        manifest['commands'].append(cmd)
        (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        with (release/(part+'-build.log')).open('x',encoding='utf-8') as f:
            result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
        if result.returncode:
            manifest.update(status='failed',failed_part=part,exit_code=result.returncode)
            (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
            raise RuntimeError(f'{part} build failed; see {release}')
    manifest['status']='built'
    manifest['artifacts']={p.relative_to(release).as_posix():digest(p)
        for part in ('native/install','interfaces/install','sensors','deps') for p in (release/part).rglob('*') if p.is_file()}
    (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print('Built candidate:',release)

if __name__=='__main__':main()
