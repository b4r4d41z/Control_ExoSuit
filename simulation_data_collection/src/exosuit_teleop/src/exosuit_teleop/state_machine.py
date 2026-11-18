"""ROS-independent arm lifecycle. An absent setpoint invokes the verified watchdog."""

from enum import Enum, auto
from threading import Event

from .trajectory import near


class State(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    CALIBRATING = auto()
    TELEOP = auto()
    HOLD = auto()
    RETURNING = auto()
    FAULT = auto()


ARM_PERIOD = 0.1


class Teleoperation:
    def __init__(self, path, safety, limits, encoders, planner):
        self.path, self.safety, self.limits = path, safety, limits
        self.encoders, self.planner = encoders, planner
        self.state = State.IDLE
        self.reason = ""
        self.pending = None
        self.segments = []
        self.command = None
        self.next_command = 0.0
        self.encoder_stamp = None
        self.hold_since = None

    def stop(self, state, now, reason):
        if self.pending:
            self.pending[2].set()
            self.planner.discard()
        self.pending = None
        self.segments = []
        self.command = None
        self.state, self.reason, self.hold_since = state, reason, now

    def fault(self, now, reason):
        self.stop(State.FAULT, now, reason)

    def _request(self, kind, start, path, now):
        cancelled = Event()
        if not self.planner.start(kind, start, path, cancelled):
            self.reason = "Previous collision check is still running"
            return False
        self.pending = kind, start, cancelled, now
        return True

    def step(self, now, feedback, a=False, encoder_sample=None):
        if self.state == State.FAULT:
            return None
        try:
            return self._step(now, feedback, a, encoder_sample)
        except (KeyError, ValueError, RuntimeError, OSError) as error:
            self.fault(now, str(error))
            return None

    def _step(self, now, fb, a, encoder_sample):
        state = self.state
        if a and state in (State.INITIALIZING, State.TELEOP, State.RETURNING, State.CALIBRATING):
            self.stop(State.HOLD, now, "Arm stream paused")
            return None

        if fb is None:
            if state not in (State.IDLE, State.HOLD):
                raise RuntimeError("Arm feedback or ROS graph is unavailable")
            if a:
                self.reason = "Fresh feedback and a healthy ROS graph are required"
            return None
        if state == State.IDLE or state == State.HOLD:
            if not a:
                return None
            self.limits.check(fb.position)
            if not fb.stopped(self.safety.stopped):
                self.reason = "Robot must be stationary"
                return None
            if state == State.IDLE:
                if not near(fb.position, self.path[0], self.safety.arrival):
                    self.reason = "Robot is outside the ZERO tolerance"
                    return None
                if self._request("initial", fb.position, self.path, now):
                    self.state, self.reason = State.INITIALIZING, ""
            else:
                if now - self.hold_since < self.safety.stop_settle:
                    self.reason = "Controller stop is still settling"
                    return None
                if self._request("rejoin", fb.position, self.path, now):
                    self.state, self.reason = State.RETURNING, ""
            return None

        self.limits.check(fb.position)
        if any(abs(v) > limit for v, limit in zip(fb.velocity, self.limits.velocity)):
            raise RuntimeError("Measured arm velocity exceeds configured limits")
        if self.command is not None and not near(fb.position, self.command, self.safety.tracking):
            raise RuntimeError("Arm following error exceeds configured tolerance")

        if state == State.CALIBRATING:
            if (not near(fb.position, self.path[-1], self.safety.arrival)
                    or not fb.stopped(self.safety.stopped)):
                raise RuntimeError("Robot moved away from READY during calibration")
            if encoder_sample is not None and encoder_sample[1] > self.calibration_since:
                ticks, stamp = encoder_sample
                self.target = self.encoders.calibrate(ticks, stamp, self.path[-1])
                self.encoder_stamp = stamp
                self.state = State.TELEOP
            elif now - self.calibration_since > self.safety.encoder_age:
                raise RuntimeError("No fresh encoder sample after reaching READY")

        if self.state == State.TELEOP:
            if encoder_sample is not None and encoder_sample[1] > self.encoder_stamp:
                ticks, stamp = encoder_sample
                self.target = self.encoders.target(ticks, stamp)
            else:
                stamp = self.encoder_stamp
            self.encoder_stamp = stamp
            if not 0 <= now - stamp <= self.safety.encoder_age:
                raise RuntimeError("Encoder sample expired")
            self.limits.check(self.target)

        if self.pending:
            kind, start, _, requested = self.pending
            if now - requested > self.safety.plan_timeout:
                raise RuntimeError("Collision validation timed out")
            if kind != "live" and (
                not near(fb.position, start, self.safety.arrival)
                or not fb.stopped(self.safety.stopped)
            ):
                raise RuntimeError("Robot moved during automatic planning")
            done, result = self.planner.poll()
            if done:
                self.pending = None
                if not result:
                    if kind == "rejoin":
                        self.stop(State.HOLD, now, "No collision-checked route to READY")
                        return None
                    raise RuntimeError("No collision-checked arm path")
                self.segments = list(result)
                self.started = None
                self.arrival_deadline = None

        if now < self.next_command:
            return None
        self.next_command = now + ARM_PERIOD

        if self.segments:
            segment = self.segments[0]
            if self.started is None:
                self.started = now
            elapsed = now - self.started
            desired = segment.at(elapsed)
            if elapsed >= segment.duration:
                if self.arrival_deadline is None:
                    # Require feedback received after the first terminal setpoint.
                    self.terminal_sent = now
                    self.terminal_feedback_stamp = fb.stamp
                    self.arrival_deadline = now + self.safety.stop_settle + self.safety.age
                elif (fb.received > self.terminal_sent
                      and (fb.stamp is None or fb.stamp > self.terminal_feedback_stamp)
                      and near(fb.position, segment.goal, self.safety.arrival)
                      and fb.stopped(self.safety.stopped)):
                    del self.segments[0]
                    self.started = self.arrival_deadline = None
                    if not self.segments and self.state != State.TELEOP:
                        self.state = State.CALIBRATING
                        self.calibration_since = now
                elif now > self.arrival_deadline:
                    raise RuntimeError("Robot did not settle at the commanded waypoint")
            if not near(fb.position, desired, self.safety.tracking):
                raise RuntimeError("Next arm command exceeds tracking tolerance")
            self.command = desired
            return desired

        if self.state == State.TELEOP and self.pending is None and self.target != self.command:
            self._request("live", self.command, (self.target,), now)
        return self.command
