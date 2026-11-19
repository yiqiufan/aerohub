"""Evaluate saved two-UAV evidence without accessing vehicles or Docker."""
import argparse,json,math,hashlib
from pathlib import Path

def cleanup_checks(root, fleet):
    """Match this run's resource identities, including the optional recorder."""
    read=lambda p:json.loads((root/p).read_text(encoding='utf-8'))
    local=read('local.json');prefix=local['project']
    roles=['adapter','native','inputs','overhead','observer']
    roles+=['camera-'+v['namespace'] for v in fleet['vehicles']]
    if local.get('diagnostics',{}).get('enabled'):roles.append('diagnostics')
    expected_workers={prefix+'-'+role for role in roles}
    expected_core={prefix+'-'+role+'-1' for role in read('compose.json')['services']}
    workers=read('cleanup-result.json');core=read('core-final.json')
    worker_names=[x.get('name','') for x in workers]
    core_names=[x.get('Name','').lstrip('/') for x in core]
    checks={
        'worker_identities':len(worker_names)==len(expected_workers) and set(worker_names)==expected_workers,
        'core_identities':len(core_names)==len(expected_core) and set(core_names)==expected_core,
        'all_exited_cleanly':all(x['State']['Running'] is False and x['State']['ExitCode']==0 for x in workers+core),
    }
    return {'pass':all(checks.values()),'checks':checks,
            'expected_workers':sorted(expected_workers),'expected_core':sorted(expected_core)}

def observed_excursion(tracking, vehicle, result):
    samples=[x['source']['vehicles'][vehicle]['physics']['position'] for x in tracking if x['event']=='airsim']
    origin=result.get('exploration_start_xy') or ([samples[0][k+'_val'] for k in 'xy'] if samples else [0.,0.])
    return max((math.dist([p[k+'_val'] for k in 'xy'],origin) for p in samples),default=0.)

def evaluate(root):
    read=lambda p:json.loads((root/p).read_text(encoding='utf-8'))
    result=read('mission/result.json');fleet=read('configuration/fleet.json');cfg=read('configuration/flight.json');flow=read('flow-result.json')
    metrics={};checks={};artifacts={}
    for v in fleet['vehicles']:
        ns=v['namespace'];events=[json.loads(s) for s in (root/f'mission/{ns}/flight-events.jsonl').read_text().splitlines()]
        tracking=[x for x in events if x['phase']=='tracking']
        adapter=[json.loads(s) for s in (root/f'adapter/{ns}/adapter-events.jsonl').read_text().splitlines()]
        distances=[]
        for x in tracking:
            if x['event']!='airsim':continue
            points=[]
            for peer in fleet['vehicles']:
                p=x['source']['vehicles'][peer['airsim']]['physics']['position']
                points.append([p[k+'_val']+peer['spawn_ned'][i] for i,k in enumerate('xyz')])
            distances.append(math.dist(*points))
        rv=result['vehicles'][ns]
        excursion=observed_excursion(tracking,v['airsim'],rv)
        complete=result.get('mode')=='explore-complete'
        motion=excursion if complete else rv.get('horizontal_displacement_m',0)
        natural=not complete or (rv.get('exploration_complete') and rv.get('termination_reason')=='native_finished')
        metrics[ns]={'active_revisions':[x['plan'] for x in adapter if x['event']=='active'],
            'rejected_count':sum(x['event']=='rejected' for x in adapter),
            'tracking_as2_references':sum(x['event']=='as2_reference' for x in tracking),
            'tracking_px4_references':sum(x['event']=='px4_setpoint' for x in tracking),
            'tracking_shared_trajectories':sum(x['event']=='shared_native_trajectory' for x in tracking),
            'tracking_wall_s':(tracking[-1]['wall_ns']-tracking[0]['wall_ns'])/1e9 if tracking else None,
            'sampled_tracking_min_separation_m':min(distances) if distances else None,
            'horizontal_displacement_m':rv.get('horizontal_displacement_m'),
            'sampled_max_exploration_displacement_m':excursion}
        checks[ns]=bool(rv.get('pass') and rv.get('stop_confirmed') and rv.get('native_land_success') and rv.get('landed_disarmed')
            and len(set(metrics[ns]['active_revisions']))>=cfg['minimum_active_revisions']
            and natural and motion>=cfg['minimum_observed_displacement_m']
            and metrics[ns]['tracking_as2_references']==rv['as2_references']>0
            and metrics[ns]['tracking_px4_references']==rv['px4_references']>0)
        for rel in [f'ulog/{ns}.ulg',f'mission/{ns}/result.json',f'adapter/{ns}/adapter-events.jsonl']:
            p=root/rel;artifacts[rel]=hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
    cleanup=cleanup_checks(root,fleet)
    checks['cleanup']=cleanup['pass']
    checks['flow']=bool(flow['pass']);checks['native_cooperation']=bool(result['pairwise_consumption'])
    for rel in ['flow-result.json','runtime-source.json','build-manifest.json','configuration/flight.json','configuration/fleet.json']:
        artifacts[rel]=hashlib.file_digest((root/rel).open('rb'),'sha256').hexdigest()
    summary={'scope':('two-UAV PC native completion; not full coverage certification' if result.get('mode')=='explore-complete' else 'two-UAV PC bounded native exploration; not complete RACER reproduction'),'pass':all(checks.values()),'checks':checks,'metrics':metrics,'cleanup':cleanup,'artifacts_sha256':artifacts}
    (root/'evaluation.json').write_text(json.dumps(summary,indent=2));return summary

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run-root',type=Path,required=True);args=ap.parse_args()
    report=evaluate(args.run_root);print(json.dumps(report,indent=2));raise SystemExit(0 if report['pass'] else 1)
