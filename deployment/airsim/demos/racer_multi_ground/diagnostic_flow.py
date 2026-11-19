"""Ground-only diagnostic collection using the normal staged entry and shadow mission."""
import argparse
import json
import subprocess
import time
from pathlib import Path
from run import Deployment, ONBOARD


def execute(d):
    stages=[]; outcome={'pass':False,'motion_requested':False,'stages':stages}
    def stage(name,operation):
        row={'stage':name,'start_wall_ns':time.time_ns()};stages.append(row)
        try:
            code=operation() or 0;row['exit']=code
            if code:raise RuntimeError(name+' returned '+str(code))
        except Exception as exc:
            row['error']=str(exc);raise
        finally:
            row['end_wall_ns']=time.time_ns()
            if d.root.exists():(d.root/'diagnostic-flow.json').write_text(json.dumps(outcome,indent=2))
    try:
        if not d.local.get('diagnostics',{}).get('enabled'):raise ValueError('explicit diagnostics.enabled required')
        if not d.local['diagnostics'].get('probe_points'):raise ValueError('configure ground probe XYZ points for this case')
        stage('up',d.up);stage('sensors',d.sensors);stage('start',d.start)
        mission_error=None
        try:stage('ground-shadow-mission',d.mission)
        except Exception as exc:mission_error=str(exc)
        outcome['ground_shadow_pass']=mission_error is None
        stage('ground-after',lambda:d.ground(d.root))
        stage('stop',d.stop);stage('down',d.down)
        # Offline reader after all writers have finalized; no DDS or flight services needed.
        def verify():
            out=d.root/'diagnostic-verification';out.mkdir()
            args=['docker','run','--rm','--network','none','--pull=never',
                  '--mount',f'type=bind,source={d.root.as_posix()},target=/evidence,readonly',
                  '--mount',f'type=bind,source={out.as_posix()},target=/result',
                  '--mount',f'type=bind,source={d.release.as_posix()},target=/release,readonly',
                  '--mount',f'type=bind,source={Path(__file__).resolve().parent.as_posix()},target=/code,readonly',
                  '--entrypoint','bash',d.images['onboard'],'-lc',ONBOARD+
                  'source /release/native/install/local_setup.bash\nexec python3 /code/verify_diagnostics.py']
            (out/'command.json').write_text(json.dumps(args,indent=2))
            with (out/'console.log').open('x') as f:code=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,timeout=120).returncode
            outcome['recording_pass']=code==0
            return code
        stage('bag-readback',verify)
        if mission_error:raise RuntimeError(mission_error)
        outcome['pass']=True
    except Exception as exc:
        outcome['error']=str(exc)
        outcome['recovery']='Inspect owned resources and actual PX4 landed/disarmed state before cleanup. Do not automatically restart or arm.'
    if d.root.exists():(d.root/'diagnostic-flow.json').write_text(json.dumps(outcome,indent=2))
    print(json.dumps(outcome,indent=2),flush=True)
    return 0 if outcome['pass'] else 1


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--local',required=True,type=Path)
    args=ap.parse_args();d=Deployment(args.local)
    if d.root.exists():raise SystemExit('Use a new run_root; preserve previous evidence')
    raise SystemExit(execute(d))
