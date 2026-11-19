"""Build a replay fixture from recorded RACER messages using the AeroHub spline evaluator."""

from collections import Counter, defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

from bspline_evaluator import BSpline, uniform_knots

NSEC = 1_000_000_000
CONTINUITY_LIMITS = (0.151, 0.501, 0.501, 0.151)


def ns(stamp):
    return NSEC * stamp["sec"] + stamp["nanosec"]


def vector(fields):
    return tuple(fields[key] for key in ("x", "y", "z"))


def angular_error(before, after):
    return abs(math.remainder(after - before, math.tau))


def errors(before, after):
    return (*[math.dist(before[i], after[i]) for i in range(3)],
            angular_error(before[3], after[3]))


@dataclass
class Plan:
    vehicle: int
    id: int
    start: int
    received: int
    position: BSpline
    velocity: BSpline
    acceleration: BSpline
    yaw: BSpline

    @classmethod
    def parse(cls, row):
        msg = row["message"]
        position = BSpline([vector(p) for p in msg["pos_pts"]], msg["order"], msg["knots"])
        velocity = position.derivative()
        yaw = BSpline([(p,) for p in msg["yaw_pts"]], 3,
                      uniform_knots(len(msg["yaw_pts"]), 3, msg["yaw_dt"]))
        return cls(row["vehicle"], msg["traj_id"], ns(msg["start_time"]),
                   row["received_ns"], position, velocity, velocity.derivative(), yaw)

    def sample(self, time):
        return (self.position.evaluate(time), self.velocity.evaluate(time),
                self.acceleration.evaluate(time), self.yaw.evaluate(min(self.yaw.duration, time))[0])

    def sample_times(self, period=0.02):
        # Subdivide each position polynomial interval so quintic reconstruction
        # does not interpolate a single segment across a B-spline knot.
        start = self.position.start
        end = self.position.end
        breaks = sorted({0.0, self.position.duration} |
                        {k - start for k in self.position.knots if start < k < end})
        for a, b in zip(breaks, breaks[1:]):
            count = math.ceil((b - a) / period)
            for index in range(count):
                yield a + (b - a) * index / count
        yield self.position.duration

