"""Separate localhost ROS processes exercising the production nodes with fake devices.

Run with a fresh roscore, a sourced Catkin workspace and no production bridge.
"""

from pathlib import Path
import runpy
import subprocess
import sys
import time
from signal import SIGINT
from unittest.mock import patch

import rospy
from dynamic_biped.msg import robotHandPosition, robotQVTau
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.safety import ARM, ARM_FEEDBACK, GRIP, RECORD, Safety
from exosuit_teleop.trajectory import Limits


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class Joy:
    def __init__(self, side):
        self.side = side
        self.button = self

    def check(self):
        pass

    def pressed(self, key):
        return int(key in rospy.get_param("/exo_test/buttons", []))

    def get_button_a(self):
        return self.pressed("A")

    def get_button_y(self):
        return self.pressed("Y")

    def get_button_x(self):
        return self.pressed("X")

    def get_control(self):
        return None, rospy.get_param(f"/exo_test/grip_{self.side}", 1), None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        pass


def test_arm_node(legacy):
    adapter = runpy.run_path(str(SCRIPTS / "exosuit_teleop_node.py"))
    rospy.init_node("exosuit_teleop")
    names = adapter["WIRE_NAMES"]

    class Sensors:
        def read(self):
            ticks = dict.fromkeys(range(1, 15), 100)
            ticks[2] = rospy.get_param("/exo_test/encoder_2", 100)
            return ticks

    def pose(first):
        return (first,) + (0.0,) * 13

    feedback_type = "dynamic_biped/robotQVTau" if legacy else "sensor_msgs/JointState"
    safety = Safety("/unused" if legacy else "", 0.25, 2.0, 0.45, 3.0, 0.01, 0.01,
                    0.3, 100, 0.4, feedback_type, "degrees", names if legacy else ())
    limits = Limits((-5.0,) * 14, (5.0,) * 14, (40.0,) * 14, (60.0,) * 14)
    # Synthetic collision model used only in this test process.
    def valid(configuration):
        return -4.0 < configuration[0] < 4.0

    adapter["Node"](safety, tuple(map(pose, (0, 0.5, 1, 1.5))), limits,
                    tuple((name, (2, 1)[i] if i < 2 else i + 1, 1)
                          for i, name in enumerate(names)), Sensors(), Joy("right"), valid if legacy else None).run()


def smoke(legacy):
    rospy.init_node("exo_test_peer")
    rospy.set_param("/exo_test/buttons", [])
    rospy.set_param("/exo_test/encoder_2", 100)
    for side in ("left", "right"):
        rospy.set_param(f"/exo_test/grip_{side}", 1)
    arms, grips, records = [], [], []
    arm_sub = rospy.Subscriber(ARM, JointState, arms.append)
    grip_sub = rospy.Subscriber(GRIP, robotHandPosition, grips.append)
    record_sub = rospy.Subscriber(RECORD, Bool, lambda msg: records.append(msg.data))
    arm_fb = rospy.Publisher(ARM_FEEDBACK, robotQVTau if legacy else JointState, queue_size=1)
    names = ["zarm_{}{}_joint".format(side, i) for side in ("r", "l") for i in range(1, 8)]
    children = {
        role: subprocess.Popen([sys.executable, "-B", __file__, role, *sys.argv[1:]])
        for role in ("--arms", "--grip", "--record")
    }

    def spin(seconds, complete=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            for role, child in children.items():
                if child.poll() is not None:
                    raise AssertionError(f"{role} exited during smoke test")
            q = list(arms[-1].position) if arms else [0.0] * 14
            stamp = rospy.Time.now()
            if legacy:
                arm_msg = robotQVTau(q=q, v=[0.0] * 14, tau=[0.0] * 14)
            else:
                arm_msg = JointState(name=names, position=q, velocity=[0.0] * 14)
                arm_msg.header.stamp = stamp
            arm_fb.publish(arm_msg)
            if complete and complete():
                return
            time.sleep(0.03)
        if complete:
            raise AssertionError("Timed out waiting for test output")

    def press(key):
        rospy.set_param("/exo_test/buttons", [key])
        if key in ("ZL", "ZR"):
            side = "left" if key == "ZL" else "right"
            param = f"/exo_test/grip_{side}"
            rospy.set_param(param, 1 - rospy.get_param(param))
        spin(0.15)
        rospy.set_param("/exo_test/buttons", [])
        spin(0.15)

    try:
        spin(5, lambda: records == [False])
        spin(5, lambda: grip_sub.get_num_connections() == 1)
        press("X")
        assert records == [False, True] and not arms
        press("Y")
        press("ZR")
        assert list(grips[-1].right_hand_position) == [0, 100, 0, 0, 0, 0]
        press("ZR")
        assert list(grips[-1].right_hand_position) == [69, 99, 42, 44, 61, 60]
        assert list(grips[-1].left_hand_position) == [0, 100, 0, 0, 0, 0]
        press("ZL")
        press("ZL")
        assert list(grips[-1].left_hand_position) == [69, 99, 42, 54, 61, 60]
        press("Y")

        def grip_payload(msg):
            return msg.header.stamp, msg.left_hand_position, msg.right_hand_position

        held_grip = grip_payload(grips[-1])
        grip_count = len(grips)
        press("ZR")
        assert len(grips) > grip_count and grip_payload(grips[-1]) == held_grip
        press("A")
        spin(7, lambda: arms and arms[-1].position[0] == 1.5)
        spin(0.4)
        assert len(arms[-1].position) == len(arms[-1].name) == 14
        rospy.set_param("/exo_test/encoder_2", 103)
        spin(5, lambda: arms[-1].position[0] > 1.5)
        press("A")
        assert records[-1] is True
        count = len(arms)
        spin(0.5)
        assert len(arms) == count
        press("A")
        spin(5, lambda: len(arms) > count and arms[-1].position[0] == 1.5)
        spin(0.4)
        competitor = rospy.Publisher(ARM, JointState, queue_size=1)
        spin(0.8)
        count = len(arms)
        spin(0.5)
        competitor.unregister()
        spin(0.5)
        assert len(arms) == count and records[-1] is True
        # Stopping one publisher leaves the other processes running.
        recorder = children.pop("--record")
        recorder.send_signal(SIGINT)
        recorder.wait(timeout=10)
        assert recorder.returncode == 0
        spin(2, lambda: records == [False, True, False])
        assert grip_payload(grips[-1]) == held_grip
        print("ROS TCPROS smoke passed: three processes, grip hold, recording, arm HOLD/return and fault")
    finally:
        for child in children.values():
            if child.poll() is None:
                child.send_signal(SIGINT)
        for child in children.values():
            child.wait(timeout=10)
        for sub in (arm_sub, grip_sub, record_sub):
            sub.unregister()


def test_peripheral_node(script):
    # The smoke runner supplies an isolated loopback master. The deployment guard
    # is exercised separately so this test can use the developer's test port.
    with patch("exosuit_teleop.joycon_input.JoyCon", Joy), patch(
        "exosuit_teleop.ros_adapter.require_simulation_master"
    ):
        runpy.run_path(str(SCRIPTS / script), run_name="__main__")


if __name__ == "__main__":
    if "--arms" in sys.argv:
        test_arm_node("--legacy" in sys.argv)
    elif "--grip" in sys.argv:
        test_peripheral_node("grip_state_publisher.py")
    elif "--record" in sys.argv:
        test_peripheral_node("isaac_record_trigger.py")
    else:
        smoke("--legacy" in sys.argv)
