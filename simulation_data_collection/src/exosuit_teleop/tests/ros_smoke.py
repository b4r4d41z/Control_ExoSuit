"""Two localhost ROS nodes exercising the production adapter with fake devices.

Run with a fresh roscore, a sourced Catkin workspace and no production bridge.
"""

from pathlib import Path
import runpy
import subprocess
import sys
import time

import rospy
from dynamic_biped.msg import robotHandPosition, robotQVTau
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.safety import ARM, ARM_FEEDBACK, GRIP, HAND_FEEDBACK, RECORD, Safety
from exosuit_teleop.trajectory import Limits


def test_node(legacy):
    adapter = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/exosuit_teleop_node.py"))
    rospy.init_node("exosuit_teleop")
    names = adapter["WIRE_NAMES"]

    class Joy:
        def read(self):
            return rospy.get_param("/exo_test/buttons", [])

    class Sensors:
        def read(self):
            ticks = dict.fromkeys(range(1, 15), 100)
            ticks[2] = rospy.get_param("/exo_test/encoder_2", 100)
            return ticks

    def pose(first):
        return (first,) + (0.0,) * 13

    feedback_type = "dynamic_biped/robotQVTau" if legacy else "sensor_msgs/JointState"
    safety = Safety("/unused", 0.25, 2.0, 0.45, 3.0, 0.01, 0.01, 0.3, 100, 0.4,
                    feedback_type, "degrees", names if legacy else ())
    limits = Limits((-5.0,) * 14, (5.0,) * 14, (40.0,) * 14, (60.0,) * 14)
    # Synthetic collision model used only in this test process.
    def valid(configuration):
        return -4.0 < configuration[0] < 4.0

    adapter["Node"](safety, tuple(map(pose, (0, 0.5, 1, 1.5))), limits,
                    tuple((name, (2, 1)[i] if i < 2 else i + 1, 1)
                          for i, name in enumerate(names)), Sensors(), Joy(), valid).run()


def smoke(legacy):
    rospy.init_node("exo_test_peer")
    rospy.set_param("/exo_test/buttons", [])
    rospy.set_param("/exo_test/encoder_2", 100)
    arms, grips, records = [], [], []
    arm_sub = rospy.Subscriber(ARM, JointState, arms.append)
    grip_sub = rospy.Subscriber(GRIP, robotHandPosition, grips.append)
    record_sub = rospy.Subscriber(RECORD, Bool, lambda msg: records.append(msg.data))
    arm_fb = rospy.Publisher(ARM_FEEDBACK, robotQVTau if legacy else JointState, queue_size=1)
    names = ["zarm_{}{}_joint".format(side, i) for side in ("r", "l") for i in range(1, 8)]
    hand_fb = rospy.Publisher(HAND_FEEDBACK, robotHandPosition, queue_size=1)
    child = subprocess.Popen([sys.executable, "-B", __file__, "--node", *sys.argv[1:]])

    def spin(seconds, complete=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if child.poll() is not None:
                raise AssertionError("Teleop exited during smoke test")
            q = list(arms[-1].position) if arms else [0.0] * 14
            stamp = rospy.Time.now()
            if legacy:
                arm_msg = robotQVTau(q=q, v=[0.0] * 14, tau=[0.0] * 14)
            else:
                arm_msg = JointState(name=names, position=q, velocity=[0.0] * 14)
                arm_msg.header.stamp = stamp
            arm_fb.publish(arm_msg)
            hand_msg = robotHandPosition(left_hand_position=[0] * 6, right_hand_position=[0] * 6)
            hand_msg.header.stamp = stamp
            hand_fb.publish(hand_msg)
            if complete and complete():
                return
            time.sleep(0.03)
        if complete:
            raise AssertionError("Timed out waiting for test output")

    def press(key):
        rospy.set_param("/exo_test/buttons", [key])
        spin(0.15)
        rospy.set_param("/exo_test/buttons", [])
        spin(0.15)

    try:
        spin(5, lambda: records == [False])
        press("X")
        assert records == [False] and not arms
        press("A")
        spin(7, lambda: arms and arms[-1].position[0] == 1.5)
        spin(0.4)
        assert len(arms[-1].position) == len(arms[-1].name) == 14
        press("Y")
        press("ZR")
        assert list(grips[-1].right_hand_position) == [69, 99, 42, 44, 61, 60]
        press("X")
        spin(2, lambda: records[-1])
        rospy.set_param("/exo_test/encoder_2", 103)
        spin(5, lambda: arms[-1].position[0] > 1.5)
        press("A")
        assert records[-1] is False
        count = len(arms)
        spin(0.5)
        assert len(arms) == count
        press("A")
        spin(5, lambda: len(arms) > count and arms[-1].position[0] == 1.5)
        spin(0.4)
        press("X")
        assert records[-1] is True
        competitor = rospy.Publisher(ARM, JointState, queue_size=1)
        spin(3, lambda: records[-1] is False)
        count = len(arms)
        spin(0.5)
        competitor.unregister()
        spin(0.5)
        assert len(arms) == count
        print("ROS TCPROS smoke passed: motion, grip, recording, HOLD/return, competing publisher")
    finally:
        child.terminate()
        child.wait(timeout=10)
        for sub in (arm_sub, grip_sub, record_sub):
            sub.unregister()


if __name__ == "__main__":
    if "--node" in sys.argv:
        test_node("--legacy" in sys.argv)
    else:
        smoke("--legacy" in sys.argv)
