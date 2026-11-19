import sys,runpy,time,json
from pathlib import Path
sys.argv=['probe','explore']
m=runpy.run_path('/repo/deployment/airsim/demos/racer_multi_flight/mission.py',run_name='probe')
m['rclpy'].init(args=[]);nodes=[m['VehicleMission'](v) for v in m['FLEET']['vehicles']]
rows={n.ns:[] for n in nodes};end=time.monotonic()+25
while time.monotonic()<end:
    for n in nodes:
        n.e.spin_once(timeout_sec=.01)
        if n.input_state:
            ages={k:(n.get_clock().now().nanoseconds-n.input_state[k])/1e9 for k in ['last_pair_ns','last_odom_ns']}
            row={'wall_ns':time.time_ns(),'ages_s':ages};rows[n.ns].append(row)
summary={ns:{k:{'min':min(x['ages_s'][k] for x in rs),'max':max(x['ages_s'][k] for x in rs),'outside':sum(not 0<=x['ages_s'][k]<=1.25 for x in rs),'samples':len(rs)} for k in ['last_pair_ns','last_odom_ns']} for ns,rs in rows.items()}
Path('/result/age-samples.json').write_text(json.dumps(rows));Path('/result/age-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
for n in nodes:n.log.close();n.e.shutdown();n.destroy_node()
m['rclpy'].shutdown()
