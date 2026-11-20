"""Named pose loading and quintic joint-space interpolation."""

from dataclasses import dataclass
from math import isfinite


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not isfinite(value) or value <= 0:
        raise ValueError("Expected a positive finite value for " + label)
    return float(value)


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
