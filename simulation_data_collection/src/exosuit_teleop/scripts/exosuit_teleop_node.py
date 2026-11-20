#!/usr/bin/env python3
"""Publish the reference arm command stream with simulation pose control."""

from contextlib import ExitStack
from threading import Lock
from time import monotonic, sleep

import rosgraph
import rospy
from dynamic_biped.msg import robotQVTau
from sensor_msgs.msg import JointState

from exosuit_teleop.encoder_reader import FeetechReaders, RelativeEncoders
from exosuit_teleop.joycon_input import JoyCon
from exosuit_teleop.ros_adapter import Job, MoveItValidity, arm_graph, require_simulation_master
from exosuit_teleop.safety import (
    ARM, ARM_FEEDBACK, Safety, decode_feedback, fresh,
)
from exosuit_teleop.state_machine import ARM_PERIOD, State, Teleoperation
from exosuit_teleop.teleop_diagnostics import event_line
from exosuit_teleop.trajectory import Limits, named_path
from exosuit_teleop.trajectory_rejoin import PathPlanner

FEEDBACK_MESSAGES = {"dynamic_biped/robotQVTau": robotQVTau, "sensor_msgs/JointState": JointState}
WIRE_NAMES = tuple("zarm_{}{}_joint".format(side, index)
                   for side in ("r", "l") for index in range(1, 8))


ARM_EVENTS = {
    State.IDLE: "CONTROLLER_READY",
    State.INITIALIZING: "ZERO_TO_READY",
    State.CALIBRATING: "READY_REACHED",
    State.TELEOP: "CONTROL_ENABLED",
    State.HOLD: "CONTROL_PAUSED",
    State.RETURNING: "RETURN_TO_READY",
    State.FAULT: "FAULT_LATCHED",
}


def log_event(component, level, state, event, detail=""):
    logger = {"INFO": rospy.loginfo, "WARN": rospy.logwarn,
              "ERROR": rospy.logerr}[level]
    logger("%s", event_line(component, level, state, event, detail))


class Node:
    def __init__(self, safety, path, limits, mapping, sensors, joycon, validity):
        self.safety, self.sensors, self.joycon = safety, sensors, joycon
        encoders = RelativeEncoders(mapping, safety.encoder_speed, safety.encoder_age)
        planner = PathPlanner(limits, safety.spacing, ARM_PERIOD, validity)
        self.control = Teleoperation(path, safety, limits, encoders, Job(planner.plan),
                                     check_live=validity is not None)
        self.graph_time = None
        self.graph_error = "Waiting for the simulation ROS master"
        self.graph = False, set()
        self.graph_job = Job(self.inspect_graph)
        self.encoder_job = Job(self.read_encoders)
        self.encoder_sample = None
        self.next_graph = self.next_encoder = 0.0
        self.feedback = None
        self.feedback_error = f"No messages received on {ARM_FEEDBACK}"
        self.feedback_lock = Lock()
        self.last_button = 0

        self.arm_pub = rospy.Publisher(ARM, JointState, queue_size=1)
        self.arm_sub = rospy.Subscriber(ARM_FEEDBACK, FEEDBACK_MESSAGES[safety.feedback_type],
                                        self.arm_feedback, queue_size=1)

    def read_encoders(self):
        stamp = monotonic()
        return self.sensors.read(), stamp

    def arm_feedback(self, msg):
        with self.feedback_lock:
            try:
                sample = decode_feedback(msg, self.safety.feedback_order, self.safety.feedback_units,
                                         WIRE_NAMES, monotonic())
            except (TypeError, ValueError, KeyError, OverflowError) as error:
                self.feedback = None
                self.feedback_error = f"Invalid {ARM_FEEDBACK} feedback: {error}"
                return
            if (self.feedback is None or sample.source != self.feedback.source
                    or sample.stamp is None or sample.stamp > self.feedback.stamp):
                self.feedback = sample
                self.feedback_error = ""

    def inspect_graph(self):
        master = rosgraph.Master(rospy.get_name())
        started = monotonic()
        graph = arm_graph(master.getSystemState(), master.getTopicTypes(), rospy.get_name(),
                          self.safety.feedback_type)
        return started, graph

    def feedback_for_control(self, now):
        try:
            done, result = self.graph_job.poll()
            if done:
                self.graph_time, self.graph = result
                self.graph_error = ""
        except RuntimeError as error:
            self.graph_time = None
            self.graph_error = f"Simulation ROS master unavailable: {error}"
        if now >= self.next_graph and self.graph_job.start():
            self.next_graph = now + min(self.safety.age / 2, 0.25)
        if self.graph_time is None or now - self.graph_time > self.safety.age:
            return None, self.graph_error or "No fresh response from the simulation ROS master"
        arm_ready, sources = self.graph
        if not arm_ready or not self.arm_pub.get_num_connections():
            return None, f"{ARM} needs a simulator subscriber and one command publisher"
        if not sources:
            return None, f"{ARM_FEEDBACK} needs one publisher of type {self.safety.feedback_type}"
        with self.feedback_lock:
            feedback, error = self.feedback, self.feedback_error
        if not fresh(feedback, monotonic(), rospy.Time.now().to_sec(), sources, self.safety.age):
            return None, error or f"Stale, future-dated, or unexpected-source feedback on {ARM_FEEDBACK}"
        return feedback, ""

    def send_arm(self, command):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = list(WIRE_NAMES)
        message.position = list(command)
        self.arm_pub.publish(message)

    def log_arm_state(self, previous):
        state, reason = self.control.state, self.control.reason
        if (state, reason) == previous:
            return
        level, event = "INFO", ARM_EVENTS[state]
        if state == State.FAULT:
            level = "ERROR"
        elif state == State.HOLD and reason == "No collision-checked route to READY":
            level, event = "WARN", "RETURN_UNAVAILABLE"
        elif state == previous[0] and reason:
            level, event = "WARN", "REQUEST_REJECTED"
        log_event("ARMS", level, state.name, event, reason)

    def tick(self):
        self.joycon.check()
        button = self.joycon.button.get_button_a()
        pressed = button == 1 and self.last_button == 0
        self.last_button = button
        now = monotonic()
        previous = self.control.state, self.control.reason

        if self.control.state in (State.CALIBRATING, State.TELEOP) and not pressed:
            try:
                done, result = self.encoder_job.poll()
                if done:
                    self.encoder_sample = result
                if now >= self.next_encoder and self.encoder_job.start():
                    self.next_encoder = now + ARM_PERIOD
            except RuntimeError as error:
                self.control.fault(now, f"Feetech read failed: {error}")
        else:
            self.encoder_job.discard()
            self.encoder_sample = None

        feedback, error = self.feedback_for_control(monotonic())
        command = self.control.step(monotonic(), feedback, pressed, self.encoder_sample, error)
        self.log_arm_state(previous)
        if command is not None and not rospy.is_shutdown():
            self.send_arm(command)

    def run(self):
        log_event("ARMS", "INFO", "IDLE", ARM_EVENTS[State.IDLE],
                  "Press A on the right Joy-Con to initialize or pause the arm stream.")
        while not rospy.is_shutdown():
            started = monotonic()
            self.tick()
            sleep(max(0, 0.02 - (monotonic() - started)))


def main():
    rospy.init_node("exosuit_teleop")
    event = "CONFIGURATION_REJECTED"
    try:
        require_simulation_master()
        mapping = rospy.get_param("~joints")
        if (tuple(row[0] for row in mapping) != WIRE_NAMES
                or sorted(row[1] for row in mapping) != list(range(1, 15))
                or any(type(row[1]) is not int or type(row[2]) is not int or abs(row[2]) != 1
                       for row in mapping)):
            raise ValueError("Invalid 14-joint wire order or encoder mapping")
        config = rospy.get_param("~safety")
        safety = Safety.load(config, WIRE_NAMES)
        limits = Limits.from_urdf(rospy.get_param("~model_urdf"), WIRE_NAMES, config["joints"])
        sequence = rospy.get_param("~zero_to_ready")
        if sequence != ["ZERO", "ELBOWS_OUT", "ARMS_UP", "READY"]:
            raise ValueError("Expected ZERO → ELBOWS_OUT → ARMS_UP → READY")
        path = named_path(WIRE_NAMES, sequence, rospy.get_param("~poses"))
        if any(path[0]):
            raise ValueError("ZERO must be all zero on the arm wire interface")
        for pose in path:
            limits.check(pose)
        validity = MoveItValidity(safety.service, WIRE_NAMES) if safety.service else None
        if validity is None:
            rospy.logwarn("MoveIt service is unset. Arm motion has no collision checking.")
        with ExitStack() as resources:
            event = "JOYCON_OPEN_FAILED"
            joycon = resources.enter_context(JoyCon("right"))
            event = "FEETECH_OPEN_FAILED"
            sensors = FeetechReaders(rospy.get_param("~left_serial"), rospy.get_param("~right_serial"))
            resources.callback(sensors.close)
            event = "CONTROLLER_STOPPED"
            Node(safety, path, limits, mapping, sensors, joycon, validity).run()
    except rospy.ROSInterruptException:
        return 0
    except Exception as error:
        log_event("ARMS", "ERROR", "FAULT", event, error)
        if error.__cause__ is not None:
            rospy.logerr("Underlying error: %s", error.__cause__)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
