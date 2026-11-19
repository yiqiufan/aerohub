"""Offline readback of ground diagnostic bags and matching native map revisions."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def rows(path):
    with path.open() as f:
        for line in f:
            if line.strip():yield json.loads(line)


def verify(root, ns):
    out=root/'diagnostics'/ns;prefix='/racer_multi/'+ns
    report=json.loads((out/'recorder-result.json').read_text())
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts=Counter();raw={};paired={};poses=set();maps={};max_points=Counter();frames={};encodings=set();sizes=set()
    arm=set();land=set();image_errors=[]
    while reader.has_next():
        topic,data,wall=reader.read_next();m=deserialize_message(data,types[topic]);counts[topic]+=1
        if hasattr(m,'header'):frames.setdefault(topic,set()).add(m.header.frame_id)
        if topic.endswith('/image') or topic==prefix+'/depth':
            encodings.add(m.encoding);sizes.add((m.width,m.height));payload=bytes(m.data)
            if m.encoding!='32FC1' or m.step!=m.width*4 or len(payload)!=m.step*m.height:
                image_errors.append([topic,stamp(m),'layout'])
            else:
                values=np.frombuffer(payload,dtype='>f4' if m.is_bigendian else '<f4')
                if not np.isfinite(values).all() or not (values>0).all():image_errors.append([topic,stamp(m),'depth values'])
            (paired if topic==prefix+'/depth' else raw)[stamp(m)]=hashlib.sha256(payload).hexdigest()
        if topic==prefix+'/camera_pose':poses.add(stamp(m))
        if topic.startswith(prefix+'/occupancy'):
            maps[topic.rsplit('/',1)[1],stamp(m)]=m.width*m.height
            max_points[topic]=max(max_points[topic],m.width*m.height)
        if topic.endswith('/vehicle_status_v4'):arm.add(m.arming_state)
        if topic.endswith('/vehicle_land_detected'):land.add(m.landed)
    del reader
    native=list(rows(root/'native'/ns/'map-query.jsonl'))
    clouds={(r['topic'],r['publish_stamp_ns']):r for r in native if r['kind']=='map_cloud'}
    cloud_versions={}
    for r in native:
        if r['kind']=='map_cloud':
            cloud_versions.setdefault((r['topic'],r['publish_stamp_ns']),set()).add((r['map_revision'],r['points']))
    updates=[r for r in native if r['kind']=='map_update'];probes=[r for r in native if r['kind']=='ground_probe']
    query_rows=[r for r in native if r['kind']=='collision_check']
    receipts=list(rows(out/'receipts.jsonl'));receipt_counts=Counter(r['topic'] for r in receipts)
    graph=list(rows(out/'publishers.jsonl'))
    unique_gids={t:{p['gid'] for r in graph for p in r['topics'].get(t,[])} for t in counts}
    pairs=set(raw)&set(paired)&poses
    required=[prefix+'/'+s for s in ['depth','camera_pose','odometry','occupancy','occupancy_local','occupancy_local_inflate']]
    required+=['/'+ns+'/sensor_measurements/front_center/depth_planar/'+s for s in ['image','camera_info']]
    checks={
        'writer_complete_without_queue_drop':bool(report['pass']),
        'all_cdr_messages_decode':counts==Counter(report['written'])==receipt_counts,
        'required_payloads_received':all(counts[t]>0 for t in required),
        'raw_and_adapted_images_valid':bool(raw and paired and not image_errors),
        'image_pose_pairs_available':bool(pairs),
        'paired_depth_preserves_raw_pixels':bool(pairs) and all(raw[s]==paired[s] for s in pairs),
        'paired_stamps_have_pose_and_raw':set(paired)<=set(raw)&poses,
        'clouds_link_to_native_revision':bool(maps) and all(k in clouds and clouds[k]['points']==v and k[1]>0 for k,v in maps.items()),
        'cloud_stamp_association_unambiguous':all(len(cloud_versions.get(k,set()))==1 for k in maps),
        'nonempty_raw_and_inflated_map':max_points[prefix+'/occupancy']>0 and max_points[prefix+'/occupancy_local_inflate']>0,
        'depth_updates_recorded':any(r['source']=='depth' and r['last_input_stamp_ns'] in paired for r in updates),
        'map_revisions_monotonic':all(b['map_revision']>a['map_revision'] for a,b in zip(updates,updates[1:])),
        'ground_probes_identified':bool(probes) and all(r['trajectory_id'] is None and r['samples'] for r in probes),
        'no_active_flight_collision_claim':not query_rows,
        'per_vehicle_publishers_unique':all(len(unique_gids.get(t,set()))==1 for t in required),
        'ground_only_status_observed':arm=={1} and land=={True},
    }
    return dict(vehicle=ns,pass_=all(checks.values()),checks=checks,counts=dict(counts),
        paired_images=len(paired),matched_image_pose_raw=len(pairs),encodings=sorted(encodings),sizes=sorted(sizes),
        image_errors=image_errors,max_cloud_points=dict(max_points),map_update_sources=dict(Counter(r['source'] for r in updates)),
        ground_probes=len(probes),active_collision_checks=len(query_rows),
        frames={k:sorted(v) for k,v in frames.items()},recorder=report,
        untested=['active-trajectory collision recording','flight load and collision avoidance','RoboPi mixed topology'])


def main():
    root=Path('/evidence');fleet=json.loads((root/'configuration/fleet.json').read_text())
    reports=[]
    for v in fleet['vehicles']:
        try:
            r=verify(root,v['namespace']);r['pass']=r.pop('pass_');reports.append(r)
        except Exception as exc:reports.append(dict(vehicle=v['namespace'],pass_=False,error=repr(exc)))
    result={'pass':all(r.get('pass',False) for r in reports),'vehicles':reports,
            'scope':'ground diagnostic acquisition/readback only; no motion commanded'}
    (Path('/result')/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));return 0 if result['pass'] else 1


if __name__=='__main__':raise SystemExit(main())
