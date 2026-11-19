"""Render per-vehicle configuration from the single verified platform defaults."""
import copy,json,shutil
from pathlib import Path
import yaml
from deployment.swarm.view import apply_overhead_view
root=Path('/result');config=root/'configuration';config.mkdir()
fleet_source=root/'fleet-input.json'
fleet=json.loads((fleet_source if fleet_source.exists() else Path('/code/fleet.json')).read_text())
(config/'fleet.json').write_text(json.dumps(fleet,indent=2))
shutil.copyfile('/single/config/fastdds_udp.xml',config/'fastdds_udp.xml')
settings=json.loads((root/'settings.json').read_text())
if set(settings['Vehicles'])!={v['airsim'] for v in fleet['vehicles']}:raise ValueError('settings/fleet mismatch')
settings=apply_overhead_view(settings,{'altitude_m':8.,'update_hz':3.})
(root/'settings.json').write_text(json.dumps(settings,indent=2))
for v in fleet['vehicles']:
    ns=v['namespace'];out=config/ns;out.mkdir()
    mapper=yaml.safe_load(Path('/single/config/mapper.yaml').read_text())
    mapper['/**']['ros__parameters']['map_ros']['frame_id']=fleet['common_frame']
    (out/'mapper.yaml').write_text(yaml.safe_dump(mapper))
    (out/'navigation.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_sim_time':True,
        'target_frame':ns+'/odom','body_frame':ns+'/base_link','odom_topic':'sensor_measurements/odom'}}}))
    (out/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{ns:{'fmu_prefix':'/'+ns,'target_system_id':v['px4_system_id']}}}))
    camera={'Vehicles':{v['airsim']:{'Cameras':settings['Vehicles'][v['airsim']]['Cameras']}}}
    (out/'camera-bridge.json').write_text(json.dumps(camera,indent=2))
print('Rendered',len(fleet['vehicles']),'PC vehicle configurations; native slots:',fleet['native_slots'])
