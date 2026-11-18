"""Validate actual Noetic message classes and the checked-in model/configuration."""

from io import BytesIO
from pathlib import Path
import unittest
from xml.etree import ElementTree

import yaml
from dynamic_biped.msg import robotHandPosition, robotQVTau
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from exosuit_teleop.safety import ARM, COMMAND_TYPES, GRIP, RECORD, decode_feedback, fresh
from exosuit_teleop.trajectory import Limits, named_path

PACKAGE = Path(__file__).resolve().parents[1]


class ROSContractTest(unittest.TestCase):
    def test_ros_wire_types_and_fields(self):
        joint = JointState(name=["zarm_r1_joint"], position=[-30.0])
        grip = robotHandPosition(left_hand_position=[0] * 6, right_hand_position=[0] * 6)
        for topic, msg in ((ARM, joint), (GRIP, grip), (RECORD, Bool(data=True))):
            self.assertEqual(msg._type, COMMAND_TYPES[topic])
            stream = BytesIO()
            msg.serialize(stream)
            recovered = type(msg)().deserialize(stream.getvalue())
            serialized_again = BytesIO()
            recovered.serialize(serialized_again)
            self.assertEqual(serialized_again.getvalue(), stream.getvalue())
        self.assertEqual(robotQVTau._type, "dynamic_biped/robotQVTau")
        self.assertEqual(robotQVTau.__slots__, ["q", "v", "tau"])
        legacy = robotQVTau(q=[0.5, 1.5], v=[0.1, 0.2], tau=[0, 0])
        legacy._connection_header = {"callerid": "/bridge"}
        feedback = decode_feedback(legacy, ("a", "b"), "degrees", ("b", "a"), 10)
        self.assertEqual(feedback.position, (1.5, 0.5))
        self.assertIsNone(feedback.stamp)
        self.assertTrue(fresh(feedback, 10.1, 0, {"/bridge"}, 0.2))
        self.assertFalse(fresh(feedback, 10.3, 0, {"/bridge"}, 0.2))

    def test_reference_model_and_launch(self):
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
        model = PACKAGE.parent / "biped_s40/urdf/biped_s40.urdf"
        # Fixtures to exercise URDF loading. Runtime limits must come from the deployment.
        limits = Limits.from_urdf(model, names, {
            name: {"velocity_deg_s": 20, "acceleration_deg_s2": 20} for name in names
        })
        path = named_path(names, data["zero_to_ready"], data["poses"])
        self.assertEqual(len(path), 4)
        self.assertEqual(path[0], (0,) * 14)
        for pose in path:
            limits.check(pose)
        launch = ElementTree.parse(PACKAGE / "launch/teleop.launch")
        self.assertEqual(len(launch.findall("node")), 1)
        self.assertEqual(launch.find("node").get("pkg"), "exosuit_teleop")
