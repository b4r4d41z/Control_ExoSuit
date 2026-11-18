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
from exosuit_teleop.trajectory import Limits, named_path
from exosuit_teleop.trajectory_rejoin import PathPlanner

OPEN = [0, 100, 0, 0, 0, 0]
CLOSED = ([69, 99, 42, 54, 61, 60], [69, 99, 42, 44, 61, 60])
FEEDBACK_MESSAGES = {"dynamic_biped/robotQVTau": robotQVTau, "sensor_msgs/JointState": JointState}
WIRE_NAMES = tuple("zarm_{}{}_joint".format(side, index)
                   for side in ("r", "l") for index in range(1, 8))


class Node:
    def __init__(self, safety, path, limits, mapping, sensors, joycons, validity):
        self.safety, self.sensors, self.joycons = safety, sensors, joycons
        encoders = RelativeEncoders(mapping, safety.encoder_speed, safety.encoder_age)
        planner = PathPlanner(limits, safety.spacing, ARM_PERIOD, validity)
        self.control = Teleoperation(path, safety, limits, encoders, Job(planner.plan))
        self.graph_time = None
        self.graph = (False, False, False, set(), set())
        self.graph_job = Job(self.inspect_graph)
        self.encoder_job = Job(self.read_encoders)
        self.encoder_sample = None
        self.next_graph = self.next_encoder = 0.0
        self.feedback = self.hand_feedback = None
        self.feedback_lock = Lock()
        self.edges = ButtonEdges()
        self.grip_enabled = False
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
        try:
            done, result = self.graph_job.poll()
            if done:
                self.graph_time, self.graph = result
        except RuntimeError as error:
            self.graph_time = None
            rospy.logwarn_throttle(2, "ROS Master check failed: %s", error)
        if now >= self.next_graph and self.graph_job.start():
            self.next_graph = now + min(self.safety.age / 2, 0.25)
        if self.graph_time is None or now - self.graph_time > self.safety.age:
            return False, False, False, set(), set()
        return self.graph

    def send_arm(self, command):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = list(WIRE_NAMES)
        message.position = list(command)
        self.arm_pub.publish(message)

    def grippers(self, keys, healthy):
        if not healthy:
            self.grip_enabled = False
            self.grip_command = None
            return
        if "Y" in keys:
            self.grip_enabled = not self.grip_enabled
        if self.grip_enabled:
            if "Y" not in keys:
                for i, trigger in enumerate(("ZL", "ZR")):
                    if trigger in keys:
                        self.grip_closed[i] = not self.grip_closed[i]
            command = robotHandPosition()
            command.header.stamp = rospy.Time.now()
            command.left_hand_position = CLOSED[0] if self.grip_closed[0] else OPEN
            command.right_hand_position = CLOSED[1] if self.grip_closed[1] else OPEN
            self.grip_command = command
        if self.grip_command is not None:
            self.grip_pub.publish(self.grip_command)

    def stop_recording(self):
        with self.record_lock:
            if self.recording:
                self.record_pub.publish(Bool(data=False))
                self.recording = False

    def update_recording(self, keys, healthy):
        if not healthy:
            self.stop_recording()
            self.record_synced = False
            return
        with self.record_lock:
            if not self.record_synced:
                self.record_pub.publish(Bool(data=False))
                self.record_synced = True
            if self.control.state == State.TELEOP and "X" in keys:
                self.recording = not self.recording
                self.record_pub.publish(Bool(data=self.recording))
            elif self.control.state != State.TELEOP and self.recording:
                self.record_pub.publish(Bool(data=False))
                self.recording = False

    def tick(self):
        now = monotonic()
        keys = self.edges.read(self.joycons.read(), now)
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
        if (self.control.state, self.control.reason) != previous:
            rospy.loginfo("%s %s", self.control.state.name, self.control.reason)

        self.update_recording(keys, record)
        if command is not None and not rospy.is_shutdown():
            self.send_arm(command)
        hand_ready = (grip and self.grip_pub.get_num_connections() and hand is not None
                      and hand_sources == {hand[0]} and 0 <= now - hand[1] <= self.safety.age
                      and 0 <= ros_now - hand[2] <= self.safety.age)
        self.grippers(keys, hand_ready and self.control.state != State.FAULT)

    def run(self):
        rospy.loginfo("IDLE. Right A: arms, Y: grip, X: record. ZL/ZR: toggle grip.")
        try:
            while not rospy.is_shutdown():
                started = monotonic()
                self.tick()
                sleep(max(0, 0.02 - (monotonic() - started)))
        finally:
            self.stop_recording()


def main():
    rospy.init_node("exosuit_teleop")
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
            joycons = JoyCons(rospy.get_param("~left_joycon"), rospy.get_param("~right_joycon"))
            resources.callback(joycons.close)
            sensors = FeetechReaders(rospy.get_param("~left_serial"), rospy.get_param("~right_serial"))
            resources.callback(sensors.close)
            Node(safety, path, limits, mapping, sensors, joycons, validity).run()
    except rospy.ROSInterruptException:
        return 0
    except Exception as error:
        rospy.logfatal("Arm stream stopped: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
