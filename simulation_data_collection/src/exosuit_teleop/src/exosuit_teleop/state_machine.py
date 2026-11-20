"""Open-loop arm state transitions and timed command publishing."""

from collections import deque
from enum import Enum, auto
from time import monotonic

from .trajectory import Segment


class State(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    TELEOP = auto()
    RETURNING = auto()
    PAUSED_AT_READY = auto()


ARM_PERIOD = 0.1


class Teleoperation:
    def __init__(self, path, duration, read_position, publish, clock=monotonic):
        self.read_position = read_position
        self.publish = publish
        self.clock = clock
        self.ready = path[-1]
        self.duration = duration
        self.initialization = tuple(
            Segment(start, goal, duration) for start, goal in zip(path, path[1:])
        )
        self.state = State.IDLE
        self.last_command = None
        self.segments = deque()
        self.segment_started = None

    def step(self, pressed=False):
        if pressed:
            if self.state == State.IDLE:
                # Preserve the reference's first-start encoder origin.
                self.read_position()
                self.segments.extend(self.initialization)
                self.state = State.INITIALIZING
            elif self.state == State.TELEOP:
                self.segments.append(Segment(self.last_command, self.ready, self.duration))
                self.state = State.RETURNING
            elif self.state == State.PAUSED_AT_READY:
                self.state = State.TELEOP

        if self.segments:
            self._advance(self.clock())
        elif self.state == State.TELEOP:
            self._send(self.read_position())

    def _send(self, position):
        command = tuple(position)
        self.publish(command)
        self.last_command = command

    def _advance(self, now):
        if self.segment_started is None:
            self.segment_started = now
        segment = self.segments[0]
        elapsed = now - self.segment_started
        self._send(segment.at(elapsed))
        if elapsed >= segment.duration:
            self.segments.popleft()
            self.segment_started = now if self.segments else None
            if not self.segments:
                self.state = (State.TELEOP if self.state == State.INITIALIZING
                              else State.PAUSED_AT_READY)
