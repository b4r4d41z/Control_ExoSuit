#!/usr/bin/env python3
"""One owner of the Joy-Cons and arm, gripper and recording publishers."""

from contextlib import ExitStack
from threading import Lock
from time import monotonic, sleep

import rosgraph
import rospy
from dynamic_biped.msg import robotHandPosition, robotQVTau
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.encoder_reader import FeetechReaders, RelativeEncoders
from exosuit_teleop.joycon_input import ButtonEdges, JoyCons
from exosuit_teleop.ros_adapter import Job, MoveItValidity, graph_health
from exosuit_teleop.safety import (
    ARM, ARM_FEEDBACK, GRIP, HAND_FEEDBACK, RECORD, Safety, decode_feedback, fresh,
)
from exosuit_teleop.state_machine import ARM_PERIOD, State, Teleoperation
from exosuit_teleop.teleop_diagnostics import event_line
from exosuit_teleop.trajectory import Limits, named_path
from exosuit_teleop.trajectory_rejoin import PathPlanner

OPEN = [0, 100, 0, 0, 0, 0]
CLOSED = ([69, 99, 42, 54, 61, 60], [69, 99, 42, 44, 61, 60])
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
    def __init__(self, safety, path, limits, mapping, sensors, joycons, validity):
        self.safety, self.sensors, self.joycons = safety, sensors, joycons
        encoders = RelativeEncoders(mapping, safety.encoder_speed, safety.encoder_age)
        planner = PathPlanner(limits, safety.spacing, ARM_PERIOD, validity)
        self.control = Teleoperation(path, safety, limits, encoders, Job(planner.plan))
        self.graph_time = self.master_connected = None
        self.graph = (False, False, False, set(), set())
        self.graph_job = Job(self.inspect_graph)
        self.encoder_job = Job(self.read_encoders)
        self.encoder_sample = None
        self.next_graph = self.next_encoder = 0.0
        self.feedback = self.hand_feedback = None
        self.feedback_lock = Lock()
        self.edges = ButtonEdges()
        self.grip_enabled = False
        self.grip_link = self.record_link = None
        self.grip_closed = [False, False]
        self.grip_command = None
        self.recording = self.record_synced = False
        self.record_lock = Lock()

        self.arm_pub = rospy.Publisher(ARM, JointState, queue_size=1)
        self.grip_pub = rospy.Publisher(GRIP, robotHandPosition, queue_size=1)
        self.record_pub = rospy.Publisher(RECORD, Bool, queue_size=1, latch=True)
        self.arm_sub = rospy.Subscriber(ARM_FEEDBACK, FEEDBACK_MESSAGES[safety.feedback_type],
                                        self.arm_feedback, queue_size=1)
        self.hand_sub = rospy.Subscriber(HAND_FEEDBACK, robotHandPosition,
                                         self.read_hand_feedback, queue_size=1)
        rospy.on_shutdown(self.stop_recording)

    def read_encoders(self):
        stamp = monotonic()
        return self.sensors.read(), stamp

    def arm_feedback(self, msg):
        with self.feedback_lock:
            try:
                sample = decode_feedback(msg, self.safety.feedback_order, self.safety.feedback_units,
                                         WIRE_NAMES, monotonic())
            except (TypeError, ValueError, KeyError, OverflowError):
                self.feedback = None
                return
            if (self.feedback is None or sample.source != self.feedback.source
                    or sample.stamp is None or sample.stamp > self.feedback.stamp):
                self.feedback = sample

    def read_hand_feedback(self, msg):
        source = (msg._connection_header or {}).get("callerid")
        stamp = msg.header.stamp.to_sec()
        with self.feedback_lock:
            old = self.hand_feedback
            if not source or stamp <= 0 or len(msg.left_hand_position) != 6 or len(msg.right_hand_position) != 6:
                self.hand_feedback = None
            elif old is None or old[0] != source or stamp > old[2]:
                self.hand_feedback = source, monotonic(), stamp

    def inspect_graph(self):
        master = rosgraph.Master(rospy.get_name())
        started = monotonic()
        graph = graph_health(master.getSystemState(), master.getTopicTypes(), rospy.get_name(),
                             self.safety.feedback_type)
        return started, graph

    def check_graph(self, now):
        error_detail = ""
        try:
            done, result = self.graph_job.poll()
            if done:
                self.graph_time, self.graph = result
        except RuntimeError as error:
            self.graph_time = None
            error_detail = str(error)
        if now >= self.next_graph and self.graph_job.start():
            self.next_graph = now + min(self.safety.age / 2, 0.25)
        unavailable = self.graph_time is None or now - self.graph_time > self.safety.age
        available = not unavailable
        if (available or error_detail or self.master_connected is not None) and available != self.master_connected:
            self.master_connected = available
            if available:
                log_event("ARMS", "INFO", self.control.state.name, "ROS_MASTER_CONNECTED")
            else:
                log_event("ARMS", "ERROR", self.control.state.name, "ROS_MASTER_UNAVAILABLE",
                          error_detail or "No fresh response from ROS Master")
        if unavailable:
            return False, False, False, set(), set()
        return self.graph

    def send_arm(self, command):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = list(WIRE_NAMES)
        message.position = list(command)
        self.arm_pub.publish(message)

    def grippers(self, keys, healthy):
        if healthy != self.grip_link:
            self.grip_link = healthy
            if healthy:
                log_event("GRIP", "INFO", "ENABLED" if self.grip_enabled else "DISABLED",
                          "GRIP_LINK_READY")
            else:
                reason = ("Arm controller in FAULT" if self.control.state == State.FAULT else
                          "Requires an exclusive command publisher, subscriber, and fresh hand feedback")
                log_event("GRIP", "WARN", "DISABLED", "GRIP_LINK_UNAVAILABLE", reason)
        if not healthy:
            self.grip_enabled = False
            self.grip_command = None
            return
        if "Y" in keys:
            self.grip_enabled = not self.grip_enabled
            if self.grip_enabled:
                log_event("GRIP", "INFO", "ENABLED", "CONTROL_ENABLED")
            else:
                log_event("GRIP", "INFO", "DISABLED", "CONTROL_DISABLED")
        changes = []
        if self.grip_enabled:
            if "Y" not in keys:
                for i, (trigger, side) in enumerate((("ZL", "LEFT"), ("ZR", "RIGHT"))):
                    if trigger in keys:
                        self.grip_closed[i] = not self.grip_closed[i]
                        action = "CLOSE" if self.grip_closed[i] else "OPEN"
                        changes.append(f"{side}_{action}_COMMAND_SENT")
            command = robotHandPosition()
            command.header.stamp = rospy.Time.now()
            command.left_hand_position = CLOSED[0] if self.grip_closed[0] else OPEN
            command.right_hand_position = CLOSED[1] if self.grip_closed[1] else OPEN
            self.grip_command = command
        if self.grip_command is not None:
            self.grip_pub.publish(self.grip_command)
            for event in changes:
                log_event("GRIP", "INFO", "ENABLED", event)

    def publish_recording(self, active):
        self.record_pub.publish(Bool(data=active))
        state, action = ("ON_REQUESTED", "START") if active else ("OFF_REQUESTED", "STOP")
        log_event("RECORD", "INFO", state, f"{action}_COMMAND_SENT")

    def stop_recording(self):
        with self.record_lock:
            if self.recording:
                self.publish_recording(False)
                self.recording = False

    def update_recording(self, keys, healthy):
        if not healthy and self.record_link is not False:
            log_event("RECORD", "WARN", "UNAVAILABLE", "RECORDER_LINK_UNAVAILABLE",
                      "Requires an exclusive command publisher and a recording subscriber")
        self.record_link = healthy
        if not healthy:
            self.stop_recording()
            self.record_synced = False
            return
        with self.record_lock:
            if not self.record_synced:
                self.publish_recording(False)
                self.record_synced = True
            if self.control.state == State.TELEOP and "X" in keys:
                self.recording = not self.recording
                self.publish_recording(self.recording)
            elif self.control.state != State.TELEOP and self.recording:
                self.publish_recording(False)
                self.recording = False

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
        now = monotonic()
        try:
            keys = self.edges.read(self.joycons.read(), now)
        except OSError as error:
            log_event("ARMS", "ERROR", "FAULT", "JOYCON_READ_FAILED", error)
            log_event("GRIP", "ERROR", "DISABLED", "JOYCON_READ_FAILED", error)
            raise
        arm, grip, record, arm_sources, hand_sources = self.check_graph(now)
        record = bool(record and self.record_pub.get_num_connections())
        previous = self.control.state, self.control.reason
        if self.control.state in (State.CALIBRATING, State.TELEOP):
            try:
                done, result = self.encoder_job.poll()
                if done:
                    self.encoder_sample = result
                if now >= self.next_encoder and self.encoder_job.start():
                    self.next_encoder = now + ARM_PERIOD
            except RuntimeError as error:
                self.control.fault(now, "Feetech read failed: " + str(error))
        else:
            self.encoder_sample = None

        feedback, hand = self.feedback, self.hand_feedback
        now = monotonic()
        ros_now = rospy.Time.now().to_sec()
        arm_ready = (arm and record and self.record_synced and self.arm_pub.get_num_connections()
                     and ros_now > 0 and fresh(feedback, now, ros_now, arm_sources, self.safety.age))
        command = self.control.step(now, feedback if arm_ready else None, "A" in keys,
                                    self.encoder_sample)
        self.log_arm_state(previous)

        self.update_recording(keys, record)
        if command is not None and not rospy.is_shutdown():
            self.send_arm(command)
        hand_ready = (grip and self.grip_pub.get_num_connections() and hand is not None
                      and hand_sources == {hand[0]} and 0 <= now - hand[1] <= self.safety.age
                      and 0 <= ros_now - hand[2] <= self.safety.age)
        self.grippers(keys, hand_ready and self.control.state != State.FAULT)

    def run(self):
        log_event("ARMS", "INFO", "IDLE", ARM_EVENTS[State.IDLE],
                  "Right A: arms, Y: grip, X: record. ZL/ZR: toggle grip.")
        log_event("GRIP", "INFO", "DISABLED", "CONTROL_READY")
        try:
            while not rospy.is_shutdown():
                started = monotonic()
                self.tick()
                sleep(max(0, 0.02 - (monotonic() - started)))
        finally:
            self.stop_recording()


def main():
    rospy.init_node("exosuit_teleop")
    event = "CONFIGURATION_REJECTED"
    try:
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
        validity = MoveItValidity(safety.service, WIRE_NAMES)
        with ExitStack() as resources:
            event = "JOYCON_OPEN_FAILED"
            joycons = JoyCons(rospy.get_param("~left_joycon"), rospy.get_param("~right_joycon"))
            resources.callback(joycons.close)
            event = "FEETECH_OPEN_FAILED"
            sensors = FeetechReaders(rospy.get_param("~left_serial"), rospy.get_param("~right_serial"))
            resources.callback(sensors.close)
            event = "CONTROLLER_STOPPED"
            Node(safety, path, limits, mapping, sensors, joycons, validity).run()
    except rospy.ROSInterruptException:
        return 0
    except Exception as error:
        if event == "JOYCON_OPEN_FAILED":
            log_event("GRIP", "ERROR", "DISABLED", event, error)
        log_event("ARMS", "ERROR", "FAULT", event, error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
