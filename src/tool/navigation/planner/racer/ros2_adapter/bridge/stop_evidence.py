"""Stop protocol compatibility; does not impose a new physical settling criterion."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StopPolicy:
    position_hold_contract: str = ""
    timeout_s: float = 6.0  # Same bound as the existing short-flight mission's stop wait.

    def __post_init__(self):
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("invalid stop timeout")


class StopEvidence:
    def __init__(self, policy, started, ros_ns, action_required):
        self.policy = policy
        self.started = started
        self.ros_ns = ros_ns
        self.action_terminal = not action_required
        self.ack_at = None
        self.failure = ""
        self.mode = None
        self.mode_at = -math.inf

    def acknowledge(self, success, now):
        if not success:
            self.failure = "stop service rejected hover"
        else:
            self.ack_at = now

    def check(self, now, ros_ns):
        if self.failure:
            return "failed", self.failure
        if now - self.started >= self.policy.timeout_s:
            self.failure = "stop protocol timeout"
            return "failed", self.failure
        if ros_ns < self.ros_ns:
            self.failure = "ROS time moved backwards during stop"
            return "failed", self.failure
        allowed = self.mode == 1 or (self.mode == 2 and bool(self.policy.position_hold_contract))
        if (self.ack_at is not None and self.action_terminal and allowed
                and self.mode_at >= self.started):
            return "confirmed", "terminal action, successful stop reply and compatible controller mode"
        return "waiting", "awaiting terminal action, stop reply and controller mode after stop request"
