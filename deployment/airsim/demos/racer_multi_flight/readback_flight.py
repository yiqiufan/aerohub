"""Case-only offline flight-recording check; no ROS publisher or control client."""
import json,sqlite3,sys
from pathlib import Path
sys.path.insert(0,'/ground-code')
from verify_diagnostics import verify,rows
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/evidence');reports=[]
for v in json.loads((root/'configuration/fleet.json').read_text())['vehicles']:
    ns=v['namespace']
    try:
        r=verify(root,ns)
        # The ground verifier's overall pass is not a flight verdict. Keep its
        # individual diagnostics, and explicitly list the three inapplicable checks.
        ground_checks=r.pop('checks');r.pop('pass_')
        exclude={'ground_probes_identified','no_active_flight_collision_claim','ground_only_status_observed'}
        checks={k:val for k,val in ground_checks.items() if k not in exclude}
        r['ground_only_checks_not_applied']={k:ground_checks[k] for k in exclude}
        r['untested']=['physical collision cause and obstacle clearance','RoboPi mixed topology','long-duration reliability']
        records=[x for x in rows(root/'native'/ns/'map-query.jsonl') if x['kind']=='collision_check']
        bag=next((root/'diagnostics'/ns/'bag').glob('*.db3'))
        db=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
        topics=list(db.execute('select id,type from topics where name=?',('/racer_execution/drone_'+str(v['racer_id'])+'/plan',)))
        ids=set()
        for tid,typ in topics:
            cls=get_message(typ)
            for (buf,) in db.execute('select data from messages where topic_id=?',(tid,)):ids.add(int(deserialize_message(buf,cls).traj_id))
        db.close()
        checks['native_collision_queries_present']=bool(records)
        checks['query_ids_have_recorded_native_plan']=bool(records) and all(x['trajectory_id'] in ids for x in records)
        checks['native_queries_have_clock_and_payload']=bool(records) and all(x.get('query_sim_ns',0)>0 and 'samples' in x and isinstance(x.get('safe'),bool) for x in records)
        r.update(checks=checks,pass_=all(checks.values()),recorded_plan_ids=sorted(ids),query_ids=sorted({x['trajectory_id'] for x in records}),
                 native_query_count=len(records),query_sample_count=sum(len(x['samples']) for x in records))
        r['pass']=r.pop('pass_');reports.append(r)
    except Exception as exc:reports.append({'vehicle':ns,'pass':False,'error':repr(exc)})
result={'pass':all(r['pass'] for r in reports),'scope':'diagnostic recording only; flight outcome evaluated independently; native plan/query correlation does not prove AS2 execution','vehicles':reports}
Path('/result/recording-result.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'pass':result['pass'],'vehicles':[{k:r.get(k) for k in ['vehicle','pass','checks','error','native_query_count']} for r in reports]},indent=2))
raise SystemExit(0 if result['pass'] else 1)
