#!/usr/bin/env python3
"""Publish the reference arm command stream with simulation pose control."""

from contextlib import ExitStack
from time import monotonic, sleep

import rospy
from sensor_msgs.msg import JointState

from exosuit_teleop.encoder_reader import EncoderAngles, FeetechReaders
from exosuit_teleop.joycon_input import JoyCon
from exosuit_teleop.state_machine import ARM_PERIOD, State, Teleoperation
from exosuit_teleop.teleop_diagnostics import event_line
from exosuit_teleop.trajectory import named_path, positive

ARM = "/kuavo_arm_traj"
WIRE_NAMES = tuple("zarm_{}{}_joint".format(side, index)
                   for side in ("r", "l") for index in range(1, 8))


ARM_EVENTS = {
    State.IDLE: "CONTROLLER_READY",
    State.INITIALIZING: "ZERO_TO_READY",
    State.TELEOP: "CONTROL_ENABLED",
    State.PAUSED_AT_READY: "PAUSED_AT_READY",
    State.RETURNING: "RETURN_TO_READY",
}


def log_event(component, level, state, event, detail=""):
    logger = {"INFO": rospy.loginfo, "WARN": rospy.logwarn,
              "ERROR": rospy.logerr}[level]
    logger("%s", event_line(component, level, state, event, detail))


class Node:
    def __init__(self, path, duration, mapping, sensors, joycon):
        self.sensors, self.joycon = sensors, joycon
        self.encoders = EncoderAngles(mapping)
        self.last_button = 0
        self.arm_pub = rospy.Publisher(ARM, JointState, queue_size=1)
        self.control = Teleoperation(path, duration, self.read_position, self.send_arm)

    def read_position(self):
        return self.encoders.update(self.sensors.read())

    def send_arm(self, command):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = list(WIRE_NAMES)
        message.position = list(command)
        self.arm_pub.publish(message)

    def tick(self):
        self.joycon.check()
        button = self.joycon.button.get_button_a()
        pressed = button == 1 and self.last_button == 0
        self.last_button = button
        previous = self.control.state
        self.control.step(pressed)
        state = self.control.state
        if state != previous:
            log_event("ARMS", "INFO", state.name, ARM_EVENTS[state])

    def run(self):
        log_event("ARMS", "INFO", "IDLE", ARM_EVENTS[State.IDLE],
                  "Press A on the right Joy-Con to initialize, return to READY, or resume.")
        while not rospy.is_shutdown():
            started = monotonic()
            self.tick()
            sleep(max(0, ARM_PERIOD - (monotonic() - started)))


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
        duration = positive(rospy.get_param("~transition_duration_s"), "transition_duration_s")
        sequence = rospy.get_param("~zero_to_ready")
        if sequence != ["ZERO", "ELBOWS_OUT", "ARMS_UP", "READY"]:
            raise ValueError("Expected ZERO → ELBOWS_OUT → ARMS_UP → READY")
        path = named_path(WIRE_NAMES, sequence, rospy.get_param("~poses"))
        if any(path[0]):
            raise ValueError("ZERO must be all zero on the arm wire interface")
        with ExitStack() as resources:
            event = "JOYCON_OPEN_FAILED"
            joycon = resources.enter_context(JoyCon("right"))
            event = "FEETECH_OPEN_FAILED"
            sensors = FeetechReaders(rospy.get_param("~left_serial"), rospy.get_param("~right_serial"))
            resources.callback(sensors.close)
            event = "CONTROLLER_STOPPED"
            Node(path, duration, mapping, sensors, joycon).run()
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
