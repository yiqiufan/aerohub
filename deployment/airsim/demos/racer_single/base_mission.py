"""Supervise a bounded Drone0 SITL short flight and preserve abort/recovery evidence."""
import json
import math
from pathlib import Path
import signal
import sys
import time

from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import PlatformInfo, PlatformStatus, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool

from remote_adapter import RemoteAdapter
NSEC = 1_000_000_000
from rosidl_runtime_py.convert import message_to_ordereddict


def xyz(p):
    return (p.x, p.y, p.z)


def ned_position(state):
    return tuple(state["pose"]["position"][axis + "_val"] for axis in "xyz")


def heading(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Mission(Node):
    def __init__(self, root, dry_run, namespace="drone0", vehicle="Drone0", system_id=135, peer_namespace="drone1", adapter_prefix=None):
        super().__init__(("racer_short_ground_gate" if dry_run else "racer_short_sitl_mission") + ("_"+namespace if namespace!="drone0" else ""),
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.namespace, self.vehicle, self.system_id = namespace, vehicle, system_id
        self.root = root
        self.dry_run = dry_run
        self.log = (root / ("ground-events.jsonl" if dry_run else "flight-events.jsonl")).open("w", buffering=1)
        self.phase = "preflight"
        self.rx = {}
        self.data = {}
        self.acks = {}
        self.anchor = self.goal = None
        self.yaw = 0
        self.air_start = None
        self.recovery = False
        self.stop_requested = False
        self.e = SingleThreadedExecutor()
        self.e.add_node(self)
        self.adapter = RemoteAdapter(self, dry_run, adapter_prefix)
        self.service_clients = []
        self.target = self.create_publisher(PoseStamped, "/racer_short/request_plan", 10)
        self.direct_land = self.create_publisher(VehicleCommand, f"/{namespace}/fmu/in/vehicle_command",
                                                 qos_profile_sensor_data)
        self.takeoff = ActionClient(self, Takeoff, f"/{namespace}/TakeoffBehavior")
        self.land = ActionClient(self, Land, f"/{namespace}/LandBehavior")
        self.px4_references = 0
        self.as2_references = 0
        self.create_subscription(VehicleCommandAck, f"/{namespace}/fmu/out/vehicle_command_ack_v{VehicleCommandAck.MESSAGE_VERSION}",
                                 self.on_ack, qos_profile_sensor_data)
        for key, msg, topic in (
            ("platform", PlatformInfo, "/drone0/platform/info"),
            ("peer", VehicleStatus, "/drone1/fmu/out/vehicle_status_v4"),
            ("px4", VehicleStatus, "/drone0/fmu/out/vehicle_status_v4"),
            ("land_detected", VehicleLandDetected, "/drone0/fmu/out/vehicle_land_detected"),
            ("odom", Odometry, "/drone0/navigation/odom"),
            ("camera", PoseStamped, "/racer_ground/camera_pose"),
            ("map", PointCloud2, "/racer_ground/occupancy"),
            ("truth", String, "/racer_landing/observation"),
            ("px4_ref", TrajectorySetpoint, "/drone0/fmu/in/trajectory_setpoint"),
            ("as2_ref", TrajectorySetpoints, "/drone0/motion_reference/trajectory"),
        ):
            topic = topic.replace("/drone0/", "/"+namespace+"/") if key != "peer" else "/"+peer_namespace+"/fmu/out/vehicle_status_v4"
            self.create_subscription(msg, topic, lambda value, key=key: self.on_data(key, value),
                                     qos_profile_sensor_data)

    def trace(self, kind, **values):
        self.log.write(json.dumps(dict(event=kind, phase=self.phase, wall_ns=time.time_ns(),
                                       sim_ns=self.get_clock().now().nanoseconds, **values)) + "\n")

    def on_data(self, key, value):
        self.rx[key] = time.monotonic()
        if key == "truth":
            row = json.loads(value.data)
            self.data[key] = dict(row, pose=row.get("vehicles", {}).get(self.vehicle, {}).get("rendered", {}))
        else:
            self.data[key] = value
        if key in ("land_detected", "px4", "platform") and self.phase in ("stopping", "landing"):
            self.trace("recovery_telemetry", topic=key, message=message_to_ordereddict(value))
        if key == "px4_ref" and self.phase == "tracking":
            self.px4_references += 1
            self.trace("px4_setpoint", pos=[float(x) for x in value.position],
                       velocity=[float(x) for x in value.velocity], px4_timestamp=value.timestamp)
        if key in ("odom", "peer") and self.phase != "preflight":
            self.trace("state_sample", topic=key, message=message_to_ordereddict(value))
        if key == "as2_ref" and self.phase == "tracking":
            self.as2_references += 1
            self.trace("as2_reference", message=message_to_ordereddict(value))
        if key == "truth" and self.phase != "preflight":
            self.trace("airsim", source=self.data[key])

    def on_ack(self, msg):
        if msg.target_system not in (0, 255, self.system_id, 1):
            return
        self.acks[msg.command] = msg.result
        self.trace("px4_ack", command=msg.command, result=msg.result, target_system=msg.target_system)

    def position(self):
        return xyz(self.data["odom"].pose.pose.position)

    def platform(self):
        return self.data.get("platform")

    def fresh(self, key, timeout):
        return time.monotonic() - self.rx.get(key, 0) <= timeout

    def check_motion(self):
        raise NotImplementedError("mission must define its declared flight envelope")

    def wait(self, condition, timeout, detail):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.e.spin_once(timeout_sec=0.02)
            if self.stop_requested and not self.recovery:
                raise RuntimeError("operator interrupted")
            self.check_motion()
            if condition():
                return
        raise RuntimeError(detail)

    def service(self, name, command, value):
        client = self.create_client(SetBool, f"/{self.namespace}/{name}")
        self.service_clients.append(client)
        self.wait(client.service_is_ready, 4, "platform service unavailable")
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done, 5, "platform service timeout")
        if not future.result().success:
            raise RuntimeError("platform service rejected " + name)
        self.wait(lambda: command in self.acks, 6, f"missing PX4 ACK {command}")
        if self.acks[command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected command {command}: {self.acks[command]}")

    def action(self, client, goal, result_field, timeout):
        self.wait(client.server_is_ready, 5, "motion action unavailable")
        self.trace("action_requested", action=result_field, goal=message_to_ordereddict(goal))
        future = client.send_goal_async(goal)
        self.wait(future.done, 5, "no action acknowledgement")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("AS2 action rejected")
        result = handle.get_result_async()
        self.trace("action_accepted", action=result_field)
        try:
            self.wait(result.done, timeout, f"AS2 {result_field} timeout")
        except BaseException:
            handle.cancel_goal_async()
            raise
        self.trace("action_result", action=result_field, status=result.result().status, success=getattr(result.result().result, result_field))
        if not getattr(result.result().result, result_field):
            raise RuntimeError("AS2 action failed " + result_field)

    def disarmed(self):
        return (self.fresh("platform", 1) and self.fresh("px4", 1)
                and not self.platform().armed and self.data["px4"].arming_state == VehicleStatus.ARMING_STATE_DISARMED)

    def px4_land(self):
        self.acks.pop(21, None)
        self.wait(lambda: self.direct_land.get_subscription_count() > 0, 5, "PX4 recovery input unavailable")
        self.direct_land.publish(VehicleCommand(timestamp=time.time_ns() // 1000,
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND, target_system=self.system_id, target_component=1,
            source_system=255, source_component=1, from_external=True,
            param4=math.nan, param5=math.nan, param6=math.nan, param7=math.nan))
        self.wait(lambda: 21 in self.acks, 6, "no PX4 native landing acknowledgement")
        if self.acks[21] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError("PX4 rejected landing")
        self.wait(lambda: self.disarmed() and self.fresh("land_detected", 1)
                  and self.data["land_detected"].landed, 75, "PX4 native landing did not disarm")

    def recover(self, result):
        self.recovery = True
        self.phase = "stopping"
        if result["motion_requested"]:
            reconcile_until = time.monotonic() + 0.65
            self.wait(lambda: time.monotonic() >= reconcile_until, 1.5, "state reconciliation timed out")
        if not self.dry_run and not self.disarmed():
            if self.adapter.goal_handle or self.adapter.pending or self.adapter.accepted_plans:
                self.adapter.begin_stop("mission recovery", operator=True)
                try:
                    self.wait(lambda: not self.adapter.stopping or bool(self.adapter.stop_failure),
                              6, "no Behavior stop confirmation")
                    if self.adapter.stop_failure:
                        raise RuntimeError(self.adapter.stop_failure)
                    result["behavior_stop_confirmed"] = True
                except Exception as exc:
                    result["stop_error"] = str(exc)
            self.phase = "landing"
            try:
                state = self.platform().status.state
                truth = self.data.get("truth", {})
                near_floor = (self.air_start and self.fresh("truth", 1) and "pose" in truth
                              and abs(ned_position(truth)[2] - self.air_start[2]) < 0.15)
                if not near_floor or state in (PlatformStatus.FLYING, PlatformStatus.TAKING_OFF, PlatformStatus.LANDING):
                    if "--native-land" in sys.argv:
                        self.px4_land()
                        result["landing_mode"] = "PX4 NAV_LAND"
                    else:
                        self.action(self.land, Land.Goal(land_speed=0.35), "land_success", 22)
                        result["landing_mode"] = "AS2 LandBehavior"
                else:
                    self.service("set_arming_state", 400, False)
                self.wait(self.disarmed, 10, "AS2 landing did not disarm")
            except Exception as exc:
                result["landing_error"] = str(exc)
                try:
                    self.px4_land()
                    result["px4_recovery_landing"] = True
                except Exception as failure:
                    result["recovery_error"] = str(failure)
        result["peer_disarmed"] = (self.fresh("peer", 1) and self.data["peer"].arming_state == VehicleStatus.ARMING_STATE_DISARMED)
        result["disarmed"] = self.disarmed()
        result["landed"] = self.fresh("land_detected", 1) and self.data["land_detected"].landed
        if self.data.get("truth") and "pose" in self.data["truth"]:
            result["final_airsim_ned"] = ned_position(self.data["truth"])
