"""Case-specific R5 cleanup adapter. Install only in the single-fault test runner."""
import json
import subprocess

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def matches(d,native):
    root=d.root
    marker=read(root/'injection-owner.json')
    injection=read(root/'native/injection.json')
    if not (marker=={'owner':d.prefix,'container':native['Id']} and
            native['Name']=='/'+d.prefix+'-native' and native['Config']['Labels'].get('aerohub.racer.owner')==d.prefix and
            injection['signal']=='SIGKILL' and injection['target']==injection['identities']['drone1']['exploration']):
        return False
    roles=['adapter','native','inputs','overhead','observer']+['camera-'+v['namespace'] for v in d.fleet['vehicles']]
    if d.local.get('diagnostics',{}).get('enabled'):roles.append('diagnostics')
    rows=read(root/'cleanup-result.json')
    expected={d.prefix+'-'+role:(1 if role=='native' else 0) for role in roles}
    if len(rows)!=len(expected) or {r['name'] for r in rows}!=set(expected):return False
    for r in rows:
        state=r['State']
        if (state['Status']!='exited' or state['Running'] or state['OOMKilled'] or state['Error']
                or state['ExitCode']!=expected[r['name']]):return False
    if native['State']!=next(r['State'] for r in rows if r['name']==native['Name'][1:]):return False
    workers=read(root/'native/worker-final.json')
    if len(workers)!=2 or {w['identity'] for w in workers}!={'drone0','drone1'}:return False
    task=read(root/'mission/result.json')
    if task['pass'] is not False:return False
    for w in workers:
        ns=w['identity']
        target=ns=='drone1'
        v=task['vehicles'][ns]
        ids=injection['identities'][ns]
        children=read(root/f'native/{ns}/native-cleanup.json')
        if (w['returncode']!=int(target) or w['exited_before_stop'] or w['error'] or w['pid']!=ids['parent']
                or len(children)!=3 or {c['pid'] for c in children}!=set(ids['children'])
                or any(c['escalated'] for c in children)
                or {c['name']:c['exit'] for c in children}!={'exploration':-9 if target else 0,'solver-1':0,'solver-2':0}
                or next(c['pid'] for c in children if c['name']=='exploration')!=ids['exploration']
                or v.get('error')!=('exploration exited -9' if target else 'peer mission failed; recover independently')
                or v['pass'] is not False or not v.get('landed_disarmed') or not v.get('native_land_success')):
            return False
    return True

def install(d):
    original=d.stop
    def stop():
        try:return original()
        except RuntimeError as exc:
            if str(exc)!='cleanup failed':raise
            native=json.loads(subprocess.check_output(['docker','inspect',d.prefix+'-native']))[0]
            if not matches(d,native):raise
            (d.root/'r5-expected-exit.json').write_text(json.dumps({
                'original_stop_error':str(exc),'native_container':native['Id'],'exit':1,
                'cause':'injected Drone1 exploration SIGKILL','mission_pass':False},indent=2))
            # The caller's original down() still performs its own live ground and ownership checks.
    d.stop=stop
