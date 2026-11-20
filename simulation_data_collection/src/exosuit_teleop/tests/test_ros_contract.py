"""Validate actual Noetic message classes and the checked-in pose configuration and ROS interfaces."""

from io import BytesIO
from pathlib import Path
import unittest
from xml.etree import ElementTree

import yaml
from dynamic_biped.msg import robotHandPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.trajectory import named_path

PACKAGE = Path(__file__).resolve().parents[1]


class ROSContractTest(unittest.TestCase):
    def test_ros_wire_types_and_fields(self):
        joint = JointState(name=["zarm_r1_joint"], position=[-30.0])
        grip = robotHandPosition(left_hand_position=[0] * 6, right_hand_position=[0] * 6)
        contracts = (
            (joint, "sensor_msgs/JointState"),
            (grip, "dynamic_biped/robotHandPosition"),
            (Bool(data=True), "std_msgs/Bool"),
        )
        for msg, expected_type in contracts:
            self.assertEqual(msg._type, expected_type)
            stream = BytesIO()
            msg.serialize(stream)
            recovered = type(msg)().deserialize(stream.getvalue())
            serialized_again = BytesIO()
            recovered.serialize(serialized_again)
            self.assertEqual(serialized_again.getvalue(), stream.getvalue())

    def test_reference_mapping_poses_and_launch(self):
        with (PACKAGE / "config/joint_mapping.yaml").open() as fp:
            joints = yaml.safe_load(fp)["joints"]
        with (PACKAGE / "config/trajectories.yaml").open() as fp:
            data = yaml.safe_load(fp)
        names = tuple(row[0] for row in joints)
        self.assertEqual(names, tuple("zarm_{}{}_joint".format(side, i)
                                     for side in ("r", "l") for i in range(1, 8)))
        self.assertEqual(tuple((row[1], row[2]) for row in joints),
                         ((2, 1), (1, -1), (3, 1), (4, 1), (5, 1), (6, 1), (7, 1),
                          (9, -1), (8, -1), (10, 1), (11, 1), (12, 1), (13, -1), (14, 1)))
        path = named_path(names, data["zero_to_ready"], data["poses"])
        self.assertEqual(data["zero_to_ready"], ["ZERO", "ELBOWS_OUT", "ARMS_UP", "READY"])
        self.assertEqual(path[0], (0,) * 14)
        self.assertEqual(path[1][:4], (0, 0, -90, -90))
        self.assertEqual(path[2][:4], (-30, 0, -90, -90))
        self.assertEqual(path[3][:4], (-30, 0, 0, -90))
        self.assertEqual(path[2][7:11], (-30, 0, 90, -90))
        launch = ElementTree.parse(PACKAGE / "launch/teleop.launch")
        self.assertEqual({node.get("type") for node in launch.findall("node")},
                         {"exosuit_teleop_node.py", "grip_state_publisher.py",
                          "isaac_record_trigger.py"})

        self.assertTrue(all(arg.get("default") is not None for arg in launch.findall("arg")))
