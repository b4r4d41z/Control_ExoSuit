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

import rosgraph
import rospy
import yaml
from dynamic_biped.msg import robotHandPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.trajectory import named_path


PACKAGE = Path(__file__).resolve().parents[1]
ARM = "/kuavo_arm_traj"
GRIP = "/control_robot_hand_position"
RECORD = "/data_collection/recording"


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


class Sensors:
    def __init__(self, left, right):
        self.reads = 0

    def read(self):
        self.reads += 1
        rospy.set_param("/exo_test/encoder_reads", self.reads)
        ticks = dict.fromkeys(range(1, 15), 100)
        ticks[2] = rospy.get_param("/exo_test/encoder_2")
        return ticks

    def close(self):
        pass


def smoke():
    rospy.init_node("exo_test_peer")
    rospy.set_param("/exo_test/buttons", [])
    rospy.set_param("/exo_test/encoder_2", 100)
    rospy.set_param("/exo_test/encoder_reads", 0)
    for side in ("left", "right"):
        rospy.set_param(f"/exo_test/grip_{side}", 1)

    with (PACKAGE / "config/joint_mapping.yaml").open() as config:
        mapping = yaml.safe_load(config)["joints"]
    with (PACKAGE / "config/trajectories.yaml").open() as config:
        poses = yaml.safe_load(config)
    settings = {
        "joints": mapping, **poses, "transition_duration_s": 0.8,
        "left_serial": "/dev/mock-left", "right_serial": "/dev/mock-right",
    }
    for key, value in settings.items():
        rospy.set_param(f"/exosuit_teleop/{key}", value)
    names = [row[0] for row in mapping]
    path = named_path(names, poses["zero_to_ready"], poses["poses"])

    arms, grips, records = [], [], []
    grip_sub = rospy.Subscriber(GRIP, robotHandPosition, grips.append)
    record_sub = rospy.Subscriber(RECORD, Bool, lambda msg: records.append(msg.data))
    observers = [grip_sub, record_sub]
    children = {}

    def start(role):
        children[role] = subprocess.Popen([sys.executable, "-B", __file__, role])

    def stop(role):
        child = children.pop(role)
        if child.poll() is None:
            child.send_signal(SIGINT)
        try:
            return child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
            raise

    def spin(seconds, complete=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            for role, child in children.items():
                if child.poll() is not None:
                    raise AssertionError(f"{role} exited during smoke test")
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
        for role in ("--arms", "--grip", "--record"):
            start(role)
        spin(5, lambda: records == [False])
        spin(5, lambda: grip_sub.get_num_connections() == 1)
        spin(5, lambda: (ARM, "sensor_msgs/JointState") in
             {tuple(topic) for topic in rospy.get_published_topics()})
        press("X")
        assert records == [False, True]
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
        # The first arm run has no command subscriber or simulator feedback.
        master = rosgraph.Master(rospy.get_name())
        _, subscribers, _ = master.getSystemState()
        assert ARM not in dict(subscribers)
        assert not any("/exosuit_teleop" in nodes for _, nodes in subscribers)
        assert rospy.get_param("/exo_test/encoder_reads") == 0
        press("A")
        spin(5, lambda: rospy.get_param("/exo_test/encoder_reads") >= 2)
        assert stop("--arms") == 0

        # Restart with a passive observer to verify the full command sequence.
        rospy.set_param("/exo_test/encoder_2", 100)
        rospy.set_param("/exo_test/encoder_reads", 0)
        arm_sub = rospy.Subscriber(ARM, JointState, arms.append)
        observers.append(arm_sub)
        start("--arms")
        spin(5, lambda: arm_sub.get_num_connections() == 1)
        spin(0.25)
        assert not arms
        rospy.set_param("/exo_test/buttons", ["A"])
        spin(2, lambda: bool(arms))
        assert tuple(arms[0].position) == path[0]
        rospy.set_param("/exo_test/encoder_2", 104)
        # Holding A through initialization must neither repeat it nor start a return.
        first_live = (4 * 360 / 4096,) + (0.0,) * 13
        spin(5, lambda: tuple(arms[-1].position) == first_live)
        rospy.set_param("/exo_test/buttons", [])
        spin(0.25)
        positions = [tuple(msg.position) for msg in arms]
        assert [positions.index(pose) for pose in path] == sorted(positions.index(pose) for pose in path)
        assert positions.index(path[-1]) < positions.index(first_live)
        assert all(list(msg.name) == names for msg in arms)

        before_return = len(arms)
        rospy.set_param("/exo_test/buttons", ["A"])
        spin(2, lambda: any(tuple(msg.position) != first_live for msg in arms[before_return:]))
        assert tuple(arms[before_return].position) == first_live
        reads = rospy.get_param("/exo_test/encoder_reads")
        rospy.set_param("/exo_test/encoder_2", 112)
        rospy.set_param("/exo_test/buttons", [])
        spin(0.15)
        rospy.set_param("/exo_test/buttons", ["A"])
        spin(2, lambda: tuple(arms[-1].position) == path[-1])
        count = len(arms)
        spin(0.25)
        assert len(arms) == count
        assert rospy.get_param("/exo_test/encoder_reads") == reads
        assert records[-1] is True

        rospy.set_param("/exo_test/buttons", [])
        spin(0.2)
        resume = (12 * 360 / 4096,) + (0.0,) * 13
        press("A")
        spin(2, lambda: tuple(arms[-1].position) == resume)
        assert tuple(arms[count].position) == resume
        assert stop("--record") == 0
        spin(2, lambda: records == [False, True, False])
        assert grip_payload(grips[-1]) == held_grip
        print("ROS TCPROS smoke passed: no simulator or feedback, timed poses, return, resume, grip and recording")
    finally:
        for role in tuple(children):
            stop(role)
        for sub in observers:
            sub.unregister()


def run_node(script):
    with patch("exosuit_teleop.joycon_input.JoyCon", Joy), patch(
        "exosuit_teleop.encoder_reader.FeetechReaders", Sensors
    ):
        runpy.run_path(str(PACKAGE / "scripts" / script), run_name="__main__")


if __name__ == "__main__":
    if "--arms" in sys.argv:
        run_node("exosuit_teleop_node.py")
    elif "--grip" in sys.argv:
        run_node("grip_state_publisher.py")
    elif "--record" in sys.argv:
        run_node("isaac_record_trigger.py")
    else:
        smoke()
