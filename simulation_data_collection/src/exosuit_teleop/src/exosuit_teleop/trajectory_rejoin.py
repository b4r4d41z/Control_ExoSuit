"""Select a reachable connector and the corresponding suffix of the reference path."""

from .trajectory import Segment, interpolate, validate_segment


def candidates(current, path):
    result = []
    for index, (start, goal) in enumerate(zip(path, path[1:])):
        direction = tuple(b - a for a, b in zip(start, goal))
        length_squared = sum(component**2 for component in direction)
        fractions = {i / 8 for i in range(1, 9)}  # Never select the initial ZERO.
        if length_squared > 0:
            projection = sum((q - a) * d for q, a, d in zip(current, start, direction))
            fraction = min(1, max(0, projection / length_squared))
            if fraction > 0:
                fractions.add(fraction)
        for fraction in fractions:
            point = interpolate(start, goal, fraction)
            if point == path[0]:
                continue
            distance = sum((q - p)**2 for q, p in zip(current, point))
            suffix = path[index + (2 if fraction == 1 else 1):]
            result.append((distance, point, suffix))
    return sorted(result, key=lambda item: item[0])


class PathPlanner:
    def __init__(self, limits, spacing, period, collision_free):
        self.limits, self.spacing, self.period = limits, spacing, period
        self.collision_free = collision_free

    def plan(self, kind, start, path, cancelled):
        cache = {}

        def checked(a, b):
            key = (a, b)
            if key not in cache:
                segment = Segment.between(a, b, self.limits, self.period)
                cache[key] = segment if self.collision_free is None or validate_segment(
                    segment, self.spacing, valid
                ) else None
            return cache[key]

        def valid(pose):
            if cancelled.is_set():
                raise RuntimeError("Collision check cancelled")
            return self.collision_free(pose)

        def route(waypoints):
            segments = []
            for a, b in zip(waypoints, waypoints[1:]):
                segment = checked(a, b)
                if segment is None:
                    return None
                segments.append(segment)
            return tuple(segments)

        if kind == "live":
            return validate_segment(Segment(start, path[0], self.period), self.spacing, valid)
        if kind == "rejoin" and self.collision_free is None:
            return route((start, path[-1]))
        if kind != "rejoin":
            return route((start,) + path)
        for _, point, suffix in candidates(start, path):
            if cancelled.is_set():
                return None
            selected = route((start, point) + suffix)
            if selected:
                return selected
        return None
