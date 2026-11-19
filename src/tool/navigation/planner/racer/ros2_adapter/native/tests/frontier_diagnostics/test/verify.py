"""Read real probe output. Does not reproduce the sampler or infer old flights."""
from pathlib import Path
import argparse,json,collections
p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path);a=p.parse_args()
read=lambda x:json.loads(x.read_text())
baseline=read(a.baseline/'baseline-views.json');off=read(a.candidate/'off-views.json');on=read(a.candidate/'on-views.json')
assert baseline==off==on, 'diagnostics changed sampler output'
assert len(on)==8
for row in on:
 assert (row['active']==1 and len(row['views'])>0) if row['case']=='visible_31' else (row['dormant']==1 and row['views']==[]),row
rows=[json.loads(x) for x in (a.candidate/'on-map.jsonl').read_text().splitlines()]
assert not any(json.loads(x)['kind'].startswith('frontier_') for x in (a.candidate/'off-map.jsonl').read_text().splitlines())
samples=[r for r in rows if r['kind']=='frontier_candidates'];assert len(samples)==10
expected=['outside_box','inflated_occupancy','near_unknown','insufficient_visible','accepted','insufficient_visible','insufficient_visible','insufficient_visible']
for i,(r,want) in enumerate(zip(samples,expected)):
 assert r['candidate_count']==len(r['candidates']) and r['candidate_count']>0
 assert r['last_input_stamp_ns']==1000+i and r['map_revision']==i+1
 assert {x['first_result'] for x in r['candidates']}=={want},(i,collections.Counter(x['first_result'] for x in r['candidates']))
 for v in r['candidates']:
  if v['visible']>=0:
   assert v['visible']+v['outside_fov']+v['blocked_inflate']+v['blocked_unknown']==r['cluster']['filtered_cells']
  else:assert v['yaw'] is None
  if i==2:assert v['unknown_witness'] is not None
  if i==3:assert v['visible']==30 and r['min_visible']==30
  if i==4:assert v['visible']==31
  if i==5:assert v['outside_fov']==31
  if i==6:assert v['blocked_unknown']==31
  if i==7:assert v['blocked_inflate']==31
life=[r for r in rows if r['kind']=='frontier_lifecycle' and r['last_input_stamp_ns']==2000]
assert [r['event'] for r in life]==['sampled_dormant','retained_dormant','removed_dormant','sampled_dormant']
uid=life[0]['cluster']['uid'];assert uid>0
assert all(r['cluster']['uid']==uid and r['cluster']['sample_cycle']==life[0]['cluster']['sample_cycle'] for r in life[:3])
assert life[-1]['cluster']['uid']!=uid
stops=[r for r in rows if r['kind']=='frontier_snapshot' and r['stage']=='execution_stop_requested']
assert len(stops)==1 and not stops[0]['active'] and len(stops[0]['dormant'])==1
assert stops[0]['dormant'][0]['uid']==life[-1]['cluster']['uid']
result={'pass':True,'scope':'synthetic input to actual native C++ libraries; not old-flight replay or AirSim validation',
 'algorithm_output_identical_baseline_off_on':True,'sampler_cases':8,'sample_events':len(samples),
 'lifecycle_events':[r['event'] for r in life], 'stop_snapshot':True,
 'candidate_counts':[r['candidate_count'] for r in samples],
 'log_bytes':(a.candidate/'on-map.jsonl').stat().st_size,'rows':len(rows)}
(a.candidate/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
