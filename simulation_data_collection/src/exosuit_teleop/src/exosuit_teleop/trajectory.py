"""Named pose loading and bounded, rest-to-rest joint-space interpolation."""

from dataclasses import dataclass
from math import ceil, degrees, isfinite, sqrt
from xml.etree import ElementTree


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not isfinite(value) or value <= 0:
        raise ValueError("Expected a positive finite value for " + label)
    return float(value)


def near(left, right, tolerance):
    return len(left) == len(right) and all(
        abs(a - b) <= tolerance for a, b in zip(left, right)
    )


@dataclass(frozen=True)
class Limits:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    @classmethod
    def from_urdf(cls, path, names, configured):
        joints = {joint.attrib["name"]: joint for joint in ElementTree.parse(path).findall("joint")}
        if set(configured) != set(names):
            raise ValueError("Supply verified velocity and acceleration for every controlled joint")
        lower, upper, velocities, accelerations = [], [], [], []
        for name in names:
            joint = joints[name]
            if joint.get("type") != "revolute":
                raise ValueError("Expected bounded revolute joint " + name)
            limit = joint.find("limit").attrib
            low, high = degrees(float(limit["lower"])), degrees(float(limit["upper"]))
            speed = positive(configured[name]["velocity_deg_s"], name)
            model_speed = positive(degrees(float(limit["velocity"])), name)
            if not isfinite(low) or not isfinite(high) or low >= high or speed > model_speed:
                raise ValueError("Inconsistent limits for " + name)
            lower.append(low)
            upper.append(high)
            velocities.append(speed)
            accelerations.append(positive(configured[name]["acceleration_deg_s2"], name))
        return cls(tuple(lower), tuple(upper), tuple(velocities), tuple(accelerations))

    def check(self, pose):
        if len(pose) != len(self.lower) or any(
            not isfinite(q) or not lo <= q <= hi for q, lo, hi in zip(pose, self.lower, self.upper)
        ):
            raise ValueError("Position outside joint limits")


def named_path(names, sequence, poses):
    if not sequence or len(sequence) != len(set(sequence)):
        raise ValueError("Trajectory needs distinct named stages")
    current = dict.fromkeys(names, 0.0)
    result = []
    for label in sequence:
        updates = poses[label]
        if not set(updates).issubset(current):
            raise ValueError("Unknown joint in pose " + label)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value)
               for value in updates.values()):
            raise ValueError("Invalid angle in pose " + label)
        current.update(updates)
        result.append(tuple(current[name] for name in names))
    return tuple(result)


@dataclass(frozen=True)
class Segment:
    start: tuple
    goal: tuple
    duration: float

    @classmethod
    def between(cls, start, goal, limits, period):
        limits.check(start)
        limits.check(goal)
        # s(u)=10u³−15u⁴+6u⁵. Peak |s'| = 15/8, peak |s''| = 10/sqrt(3).
        duration = period
        for a, b, v, acc in zip(start, goal, limits.velocity, limits.acceleration):
            travel = abs(b - a)
            duration = max(duration, 1.875 * travel / v, sqrt(10 * travel / (sqrt(3) * acc)))
        if not isfinite(duration):
            raise ValueError("Trajectory duration overflow")
        return cls(start, goal, duration)

    def at(self, elapsed):
        if elapsed <= 0:
            return self.start
        if elapsed >= self.duration:
            return self.goal
        u = elapsed / self.duration
        return interpolate(self.start, self.goal, u**3 * (10 + u * (-15 + 6 * u)))


def interpolate(start, goal, fraction):
    if fraction == 0:
        return start
    if fraction == 1:
        return goal
    return tuple(a + fraction * (b - a) for a, b in zip(start, goal))


def validate_segment(segment, spacing, valid):
    # Quintic interpolation stays on the same geometric line. Sample in joint
    # space so densification does not depend on the duration of the move.
    travel = max(abs(a - b) for a, b in zip(segment.start, segment.goal))
    steps = max(1, ceil(travel / spacing))
    if steps > 4096:
        raise ValueError("Too many collision samples for one segment")
    return all(valid(interpolate(segment.start, segment.goal, i / steps))
               for i in range(steps + 1))
