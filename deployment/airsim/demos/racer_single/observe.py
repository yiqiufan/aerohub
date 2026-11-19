"""Observe the short landing experiment without consuming AirSim collision events."""
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from std_msgs.msg import String
from uav_as2_sensor_adapter.rpc import Rpc


def main():
    root = Path("/result")
    rpc = Rpc(os.environ.get("AIRSIM_HOST", "host.docker.internal"), int(os.environ.get("AIRSIM_RPC_PORT",41461)), .8)
    vehicles = rpc.call("listVehicles")
    if sorted(vehicles) != ["Drone0", "Drone1"]:
        raise RuntimeError(f"unexpected vehicles {vehicles}")
    (root / "runtime-settings.json").write_text(rpc.call("getSettingsString"))
    rclpy.init()
    node = rclpy.create_node("racer_landing_nonconsuming_observer")
    pub = node.create_publisher(String, "/racer_landing/observation", 10)
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with (root / "observation.jsonl").open("x", buffering=1) as log:
        def tick():
            row = {"wall_ns": time.time_ns(), "vehicles": {}}
            try:
                for vehicle in vehicles:
                    row["vehicles"][vehicle] = {
                        "rendered": rpc.call("simGetVehiclePose", vehicle),
                        "physics": rpc.call("simGetGroundTruthKinematics", vehicle),
                        "gps": rpc.call("getGpsData", "", vehicle),
                        "barometer": rpc.call("getBarometerData", "", vehicle)}
            except Exception as error:
                row["error"] = str(error)
            row["wall_ns_end"] = time.time_ns()
            log.write(json.dumps(row) + "\n")
            pub.publish(String(data=json.dumps(row)))
        node.create_timer(.2, tick)
        while running:
            rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
