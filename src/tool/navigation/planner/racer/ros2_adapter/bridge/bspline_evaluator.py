"""Validation and deterministic sampling of FALCON B-spline messages."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


class InvalidSpline(ValueError):
    pass


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


@dataclass(frozen=True)
class Sample:
    time_from_start: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float


def _lerp(left, right, fraction):
    return tuple(a + fraction * (b - a) for a, b in zip(left, right))


def _yaw_delta(left, right):
    return (right - left + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_samples(samples: Sequence[Sample], time_from_start: float) -> Sample:
    if not samples or not math.isfinite(time_from_start):
        raise InvalidSpline("invalid continuity sample")
    if time_from_start < -1e-9 or time_from_start > samples[-1].time_from_start + 1e-9:
        raise InvalidSpline("replacement starts outside previous trajectory coverage")
    time_from_start = min(max(time_from_start, 0.0), samples[-1].time_from_start)
    before = after = samples[0]
    for candidate in samples:
        if candidate.time_from_start >= time_from_start:
            after = candidate
            break
        before = candidate
    span = after.time_from_start - before.time_from_start
    fraction = 0.0 if span <= 0 else (time_from_start - before.time_from_start) / span
    return Sample(
        time_from_start,
        _lerp(before.position, after.position, fraction),
        _lerp(before.velocity, after.velocity, fraction),
        _lerp(before.acceleration, after.acceleration, fraction),
        before.yaw + fraction * _yaw_delta(before.yaw, after.yaw),
    )


CONTINUITY_NUMERICAL_EPSILON = 1e-3


def exceeds_continuity_limits(deltas, limits):
    return any(delta > limit + CONTINUITY_NUMERICAL_EPSILON
               for delta, limit in zip(deltas, limits))


def continuity_deltas(previous: Sequence[Sample], previous_start: float,
                      replacement: Sequence[Sample], replacement_start: float):
    if not replacement:
        raise InvalidSpline("replacement trajectory is empty")
    expected = interpolate_samples(previous, replacement_start - previous_start)
    actual = replacement[0]
    norm = lambda values: math.sqrt(sum(value * value for value in values))
    difference = lambda left, right: tuple(a - b for a, b in zip(left, right))
    return (
        norm(difference(actual.position, expected.position)),
        norm(difference(actual.velocity, expected.velocity)),
        norm(difference(actual.acceleration, expected.acceleration)),
        abs(_yaw_delta(expected.yaw, actual.yaw)),
    )


class BSpline:
    def __init__(self, control_points: Sequence[Sequence[float]], degree: int,
                 knots: Sequence[float]):
        self.points = tuple(tuple(float(value) for value in point) for point in control_points)
        self.degree = int(degree)
        self.knots = tuple(float(value) for value in knots)
        if not 1 <= self.degree <= 7:
            raise InvalidSpline("degree must be between 1 and 7")
        if len(self.points) <= self.degree:
            raise InvalidSpline("insufficient control points")
        dimension = len(self.points[0])
        if dimension == 0 or any(len(point) != dimension for point in self.points):
            raise InvalidSpline("inconsistent control-point dimensions")
        if not _finite(value for point in self.points for value in point):
            raise InvalidSpline("control points must be finite")
        if len(self.knots) != len(self.points) + self.degree + 1:
            raise InvalidSpline("knot count does not match degree and control points")
        if not _finite(self.knots) or any(b < a for a, b in zip(self.knots, self.knots[1:])):
            raise InvalidSpline("knots must be finite and nondecreasing")
        self.start = self.knots[self.degree]
        self.end = self.knots[len(self.points)]
        if self.end <= self.start:
            raise InvalidSpline("spline has no positive duration")

    @property
    def duration(self) -> float:
        return self.end - self.start

    def evaluate(self, time_from_start: float) -> tuple[float, ...]:
        value = float(time_from_start) + self.start
        if not math.isfinite(value) or value < self.start - 1e-9 or value > self.end + 1e-9:
            raise InvalidSpline("sample is outside spline coverage")
        value = min(max(value, self.start), self.end)
        span = self.degree
        while span + 1 < len(self.knots) and self.knots[span + 1] < value:
            span += 1
        span = min(span, len(self.points) - 1)
        work = [list(self.points[span - self.degree + index])
                for index in range(self.degree + 1)]
        for level in range(1, self.degree + 1):
            for index in range(self.degree, level - 1, -1):
                left = self.knots[index + span - self.degree]
                right = self.knots[index + 1 + span - level]
                if right <= left:
                    raise InvalidSpline("degenerate active knot interval")
                alpha = (value - left) / (right - left)
                work[index] = [
                    (1.0 - alpha) * before + alpha * after
                    for before, after in zip(work[index - 1], work[index])
                ]
        result = tuple(work[self.degree])
        if not _finite(result):
            raise InvalidSpline("nonfinite spline sample")
        return result

    def derivative(self) -> "BSpline":
        points = []
        for index in range(len(self.points) - 1):
            denominator = self.knots[index + self.degree + 1] - self.knots[index + 1]
            if denominator <= 0:
                raise InvalidSpline("degenerate derivative knot interval")
            points.append(tuple(
                self.degree * (after - before) / denominator
                for before, after in zip(self.points[index], self.points[index + 1])
            ))
        return BSpline(points, self.degree - 1, self.knots[1:-1]) if self.degree > 1 else _ConstantDerivative(points, self.start, self.end)


class _ConstantDerivative:
    def __init__(self, points, start, end):
        if not points:
            raise InvalidSpline("missing derivative control point")
        self.value = tuple(points[0])
        self.start = start
        self.end = end

    def evaluate(self, time_from_start):
        value = float(time_from_start) + self.start
        if value < self.start - 1e-9 or value > self.end + 1e-9:
            raise InvalidSpline("sample is outside derivative coverage")
        return self.value


def uniform_knots(control_count: int, degree: int, interval: float) -> tuple[float, ...]:
    if control_count <= degree or not math.isfinite(interval) or interval <= 0:
        raise InvalidSpline("invalid uniform spline dimensions")
    return tuple((index - degree) * interval for index in range(control_count + degree + 1))


def sample_message(message, period: float = 0.02, max_points: int = 10000) -> tuple[Sample, ...]:
    if not math.isfinite(period) or period <= 0:
        raise InvalidSpline("sample period must be positive and finite")
    position = BSpline(
        [(point.x, point.y, point.z) for point in message.pos_pts],
        int(message.order), message.knots)
    velocity = position.derivative()
    acceleration = velocity.derivative()
    yaw = BSpline([(value,) for value in message.yaw_pts], 3,
                  uniform_knots(len(message.yaw_pts), 3, float(message.yaw_dt)))
    count = int(math.ceil(position.duration / period))
    if count < 1 or count + 1 > max_points:
        raise InvalidSpline("trajectory sample count is outside limits")
    times = [index * period for index in range(count)] + [position.duration]
    output = []
    for time_from_start in times:
        p = position.evaluate(time_from_start)
        v = velocity.evaluate(time_from_start)
        a = acceleration.evaluate(time_from_start)
        # FALCON's execution horizon is defined by the position spline. Its
        # yaw optimizer may produce a shorter or longer valid spline; upstream
        # holds the terminal yaw once its own horizon is exhausted.
        heading = yaw.evaluate(min(time_from_start, yaw.duration))[0]
        heading = (heading + math.pi) % (2.0 * math.pi) - math.pi
        output.append(Sample(time_from_start, p, v, a, heading))
    return tuple(output)
