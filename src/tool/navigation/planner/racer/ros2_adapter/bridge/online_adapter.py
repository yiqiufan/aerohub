"""Per-vehicle ROS contract prototype for RACER -> FollowTimedPath."""

from dataclasses import dataclass
import time
import uuid

from as2_msgs.msg import ControlMode, ControllerInfo
from bspline.msg import Bspline
from exploration_manager.msg import TrajectoryExecution
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedTrajectoryPoint

from trajectory import NSEC, Plan
from stop_evidence import StopEvidence, StopPolicy


def set_time(stamp, nanoseconds):
    stamp.sec, stamp.nanosec = divmod(nanoseconds, NSEC)


def set_xyz(message, values):
    message.x, message.y, message.z = values


@dataclass
class Submission:
    plan: Plan
    base: int
    initial: bool
    future: object
    fault: str


class OnlineAdapter(Node):
    def __init__(self, vehicle, namespace, speed, trace, stop_policy=None):
        super().__init__("racer_online_adapter")
        self.vehicle = vehicle
        self.namespace = namespace
        self.speed = speed
        self.trace = trace
        self.session = ""
        self.pending = None
        self.goal_handle = None
        self.result = None
        self.committed = 0
        self.active = 0
        self.stopping = False
        self.disabled = False
        self.stop_requested_ns = 0
        self.stop_result = None
        self.stop_detail = ""
        self.mode = None
        self.mode_received_ns = 0
        self.operator_stop = False
        self.last_seen_plan = 0
        self.cancel_sent = False
        self.cancel_future = None
        self.stop_policy = stop_policy or StopPolicy()
        self.stop_evidence = None
        self.stop_failure = ""
        self.stop_ack_seen = False
        prefix = f"/racer_execution/drone_{vehicle}"
        path = f"/{namespace}/follow_timed_path"
        self.action = ActionClient(self, FollowTimedPath, path)
        self.modify = self.create_client(FollowTimedPath_SendGoal, f"{path}/_behavior/modify")
        self.stop = self.create_client(Trigger, f"{path}/_behavior/stop")
        self.status = self.create_publisher(TrajectoryExecution, f"{prefix}/status", 10)
        self.create_subscription(Bspline, f"{prefix}/plan", self.on_plan, 10)
        self.create_subscription(Empty, f"{prefix}/stop", self.on_stop, 10)
        self.create_subscription(ControllerInfo, f"/{namespace}/controller/info", self.on_mode,
                                 qos_profile_sensor_data)
        self.create_timer(0.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def ready(self):
        return (self.action.server_is_ready() and self.modify.service_is_ready() and
                self.stop.service_is_ready())

    def report(self, identity, status, detail):
        self.status.publish(TrajectoryExecution(traj_id=identity, status=status, detail=detail))

    def on_mode(self, msg):
        self.mode = msg.input_control_mode.control_mode
        self.mode_received_ns = self.now_ns()
        if self.stop_evidence:
            self.stop_evidence.mode = self.mode
            self.stop_evidence.mode_at = time.monotonic()

    def on_stop(self, _):
        self.begin_stop("planner requested stop")

    def modify_wire(self, plan, goal):
        """The live test subclass uses this hook to inject a faulty request."""
        return ""

    def accept(self, pending):
        self.report(pending.plan.id, TrajectoryExecution.ACCEPTED, "behavior accepted")

    def on_plan(self, msg):
        if self.disabled or self.stopping:
            return
        if msg.drone_id != self.vehicle or msg.traj_id <= self.last_seen_plan:
            self.report(msg.traj_id, TrajectoryExecution.REJECTED, "invalid vehicle or sequence")
            return
        self.last_seen_plan = msg.traj_id
        if self.pending or not self.ready():
            self.report(msg.traj_id, TrajectoryExecution.REJECTED, "execution busy or unavailable")
            return
        try:
            plan = Plan.parse({"vehicle": self.vehicle, "received_ns": self.now_ns(),
                               "message": message_to_ordereddict(msg)})
            initial = self.goal_handle is None
            if initial:
                self.session = uuid.uuid4().hex
            goal = self.goal_for(plan)
        except (ValueError, OverflowError) as error:
            self.report(msg.traj_id, TrajectoryExecution.REJECTED, str(error))
            return
        fault = self.modify_wire(plan, goal)
        self.trace("submitted", plan=plan.id, base=self.committed, initial=initial, fault=fault,
                   start_ns=plan.start, wire_start_ns=goal.path.valid_from.sec * NSEC +
                   goal.path.valid_from.nanosec, at_ns=self.now_ns())
        if initial:
            future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
        else:
            request = FollowTimedPath_SendGoal.Request()
            request.goal = goal
            future = self.modify.call_async(request)
        self.pending = Submission(plan, self.committed, initial, future, fault)

    def goal_for(self, plan):
        if plan.start <= self.now_ns():
            raise ValueError("late before submission")
        goal = FollowTimedPath.Goal()
        path = goal.path
        path.interface_version = 1
        path.task_id = self.session
        path.vehicle_id = self.namespace
        path.frame_id = f"{self.namespace}/odom"
        path.revision = plan.id
        path.generated_at = self.get_clock().now().to_msg()
        set_time(path.valid_from, plan.start)
        path.max_speed = self.speed
        path.valid_fields = 7
        for elapsed in plan.sample_times():
            point = TimedTrajectoryPoint()
            set_time(point.time_from_start, round(elapsed * NSEC))
            position, velocity, acceleration, point.yaw = plan.sample(elapsed)
            set_xyz(point.position, position)
            set_xyz(point.velocity, velocity)
            set_xyz(point.acceleration, acceleration)
            path.points.append(point)
        return goal

    def on_feedback(self, message):
        feedback = message.feedback
        if feedback.task_id == self.session and not feedback.waiting_for_start and feedback.revision > self.active:
            self.active = feedback.revision
            self.trace("active", plan=self.active, at_ns=self.now_ns())

    def finish_submission(self):
        pending = self.pending
        self.pending = None
        response = pending.future.result()
        if response.accepted:
            if pending.initial:
                self.goal_handle = response
                self.result = response.get_result_async()
            self.committed = pending.plan.id
            self.trace("accepted", plan=pending.plan.id, base=pending.base, fault=pending.fault,
                       start_ns=pending.plan.start, at_ns=self.now_ns())
            if not self.stopping:
                self.accept(pending)
        else:
            if self.stopping and pending.initial:
                self.stop_evidence.action_terminal = True
            self.trace("rejected", plan=pending.plan.id, base=pending.base, fault=pending.fault,
                       at_ns=self.now_ns())
            if not self.stopping:
                self.report(pending.plan.id, TrajectoryExecution.REJECTED,
                            "FollowTimedPath rejected the request")

    def begin_stop(self, reason, operator=False):
        if operator:
            self.disabled = True
            self.operator_stop = True
        if self.stopping:
            return
        self.stopping = True
        self.stop_detail = reason
        self.stop_requested_ns = self.now_ns()
        self.stop_evidence = StopEvidence(self.stop_policy, time.monotonic(), self.stop_requested_ns,
                                          bool(self.goal_handle or self.pending or self.result))
        self.stop_ack_seen = False
        self.stop_failure = ""
        self.trace("stop_requested", reason=reason, operator=operator, at_ns=self.stop_requested_ns)

    def tick(self):
        if self.pending and self.pending.future.done():
            self.finish_submission()
        if self.result and self.result.done():
            action_result = self.result.result()
            self.trace("action_result", status=action_result.status,
                       reason=action_result.result.reason, plan=action_result.result.revision,
                       at_ns=self.now_ns())
            self.goal_handle = self.result = None
            self.begin_stop(action_result.result.reason)
            self.stop_evidence.action_terminal = action_result.status in (4, 5, 6)
        if not self.stopping or self.stop_failure:
            return
        state, detail = self.stop_evidence.check(time.monotonic(), self.now_ns())
        if state == "failed":
            self.stop_failure = detail
            self.disabled = True
            self.trace("stop_failed", reason=detail, plan=self.active, at_ns=self.now_ns())
            return  # Keep stopping latched; never issue STOPPED or accept another plan.
        if self.pending:
            return
        if self.goal_handle:
            if not self.cancel_sent:
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.cancel_sent = True
            if not self.cancel_future.done():
                return
            if self.cancel_future.exception() is None and self.cancel_future.result().goals_canceling:
                return  # Await the terminal Action result; rejection falls back to explicit stop.
        if not self.stop_result:
            if self.stop.service_is_ready():
                self.stop_result = self.stop.call_async(Trigger.Request())
            return
        if self.stop_result.done() and not self.stop_ack_seen:
            self.stop_ack_seen = True
            error = self.stop_result.exception()
            success = error is None and self.stop_result.result().success
            self.stop_evidence.acknowledge(success, time.monotonic())
            self.trace("stop_reply", success=success, error=str(error) if error else "",
                       message=self.stop_result.result().message if not error else "",
                       at_ns=self.now_ns())
            return
        if state == "confirmed":
            status = TrajectoryExecution.OPERATOR_STOPPED if self.operator_stop else TrajectoryExecution.STOPPED
            self.trace("stop_confirmed", plan=self.active, reason=self.stop_detail,
                       operator=self.operator_stop, mode=self.mode,
                       hold_contract=self.stop_policy.position_hold_contract,
                       evidence=detail, at_ns=self.now_ns())
            self.report(self.active, status, self.stop_detail)
            self.stopping = self.cancel_sent = self.operator_stop = False
            self.stop_result = None
            self.cancel_future = self.stop_evidence = None
            self.committed = self.active = 0
