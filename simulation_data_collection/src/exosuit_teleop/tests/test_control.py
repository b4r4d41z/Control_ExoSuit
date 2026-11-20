import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from exosuit_teleop.encoder_reader import EncoderAngles, FeetechReaders, tick_delta
from exosuit_teleop.joycon_input import JoyCon
from exosuit_teleop.state_machine import State, Teleoperation
from exosuit_teleop.trajectory import Segment, named_path, positive


class EncodersTest(unittest.TestCase):
    def test_reference_wrap_and_signed_mapping(self):
        self.assertEqual(tick_delta(1, 4095), 2)
        self.assertEqual(tick_delta(4095, 1), -2)
        self.assertEqual(tick_delta(2048, 0), 2048)
        self.assertEqual(tick_delta(0, 2048), -2048)
        for tick in (-1, 4096):
            with self.assertRaises(ValueError):
                tick_delta(tick, 0)

        encoder = EncoderAngles([("r1", 2, 1), ("r2", 1, -1)])
        self.assertEqual(encoder.update({1: 4095, 2: 4}), (0, 0))
        self.assertEqual(encoder.update({1: 1, 2: 7}), (3 * 360 / 4096, -2 * 360 / 4096))
        self.assertEqual(encoder.update({1: 4095, 2: 5}), (360 / 4096, 0))

    def test_group_reads_verify_both_buses_and_close_ports(self):
        missing = [None]
        ports, groups = [], []

        class Port:
            def __init__(self, path):
                self.path, self.is_open = path, False
                ports.append(self)

            def openPort(self):
                self.is_open = True
                return True

            def closePort(self):
                self.is_open = False

            def setPacketTimeoutMillis(self, timeout):
                self.timeout = timeout

        class Group:
            def __init__(self, port, _, address, size):
                self.path, self.address, self.size = port.path, address, size
                self.ids = []
                groups.append(self)

            def addParam(self, encoder):
                self.ids.append(encoder)

            def txRxPacket(self):
                return 0

            def isAvailable(self, encoder, address, size):
                return encoder in self.ids and encoder != missing[0] and (
                    address, size) == (self.address, self.size)

            def getData(self, encoder, address, size):
                return encoder

        sdk = SimpleNamespace(PortHandler=Port, COMM_SUCCESS=0, GroupSyncRead=Group,
                              PacketHandler=lambda protocol: protocol)
        with patch.dict(sys.modules, {"scservo_sdk": sdk}):
            sensor = FeetechReaders("/tmp/left", "/tmp/right")
            self.addCleanup(sensor.close)
            self.assertEqual(sensor.read(), {i: i for i in range(1, 15)})
            self.assertEqual([(g.path, g.address, g.size, g.ids) for g in groups],
                             [("/tmp/left", 56, 2, list(range(1, 8))),
                              ("/tmp/right", 56, 2, list(range(8, 15)))])
            missing[0] = 9
            with self.assertRaisesRegex(OSError, "encoder 9 on /tmp/right"):
                sensor.read()
            sensor.close()
            self.assertTrue(all(not port.is_open for port in ports))
            with self.assertRaisesRegex(OSError, "buses are closed"):
                sensor.read()

    def test_joycon_rejects_stale_reports_and_closes_all_handles(self):
        from tempfile import TemporaryDirectory
        from unittest.mock import Mock

        reader = SimpleNamespace(_input_report=object(),
                                 _update_input_report_thread=SimpleNamespace(is_alive=lambda: True),
                                 _close=Mock())
        gyro = SimpleNamespace(_close=Mock())
        device = SimpleNamespace(button=reader, gyro=gyro, running=True, disconnnect=Mock())
        factory = Mock(return_value=device)
        with TemporaryDirectory() as ros_home, patch.dict(
            sys.modules, {"joyconrobotics": SimpleNamespace(JoyconRobotics=factory)}
        ), patch("exosuit_teleop.joycon_input.get_ros_home", return_value=ros_home), patch(
            "exosuit_teleop.joycon_input.monotonic", side_effect=[0.0, 0.1, 0.7]
        ):
            with JoyCon("right") as joystick:
                joystick.check()
                with self.assertRaisesRegex(OSError, "stopped delivering"):
                    joystick.check()

        factory.assert_called_once_with("right", without_rest_init=True)
        self.assertFalse(device.running)
        for close in (device.disconnnect, reader._close, gyro._close):
            close.assert_called_once()



PATH = ((0.0, 0.0), (0.0, 8.0), (10.0, 8.0), (10.0, 10.0))


class MovementTest(unittest.TestCase):
    def test_cumulative_pose_validation(self):
        self.assertEqual(named_path(("a", "b"), ("ZERO", "ELBOW", "READY"),
                                    {"ZERO": {}, "ELBOW": {"b": 8}, "READY": {"a": 9}}),
                         ((0, 0), (0, 8), (9, 8)))
        for bad in (False, "invalid", 0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                positive(bad, "transition_duration_s")

    def test_quintic_has_exact_endpoints_and_zero_endpoint_speed(self):
        move = Segment((-10.0, 0.0), (10.0, 16.0), 4.0)
        self.assertEqual(move.at(-1), move.start)
        self.assertEqual(move.at(2), (0.0, 8.0))
        self.assertEqual(move.at(5), move.goal)
        step = 0.001
        for start, first, last, goal in zip(move.start, move.at(step), move.at(4 - step), move.goal):
            self.assertLess(abs(first - start) / step, 1e-4)
            self.assertLess(abs(goal - last) / step, 1e-4)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.sample = {1: 4095, 2: 100}
        self.commands = []
        self.reads = 0
        self.encoders = EncoderAngles([("a", 2, 1), ("b", 1, -1)])
        self.control = Teleoperation(PATH, 2, self.position, self.commands.append,
                                     clock=lambda: self.now)

    def position(self):
        self.reads += 1
        return self.encoders.update(self.sample)

    def step(self, now, pressed=False):
        self.now = now
        self.control.step(pressed)

    def initialize(self):
        self.step(1, pressed=True)
        self.sample = {1: 1, 2: 104}
        self.step(3)
        self.step(5)
        self.step(7)

    def test_a_starts_and_elapsed_time_publishes_every_waypoint(self):
        self.step(0)
        self.assertEqual(self.control.state, State.IDLE)
        self.assertEqual(self.commands, [])
        self.assertEqual(self.reads, 0)
        self.step(1, pressed=True)
        self.assertEqual(self.control.state, State.INITIALIZING)
        self.step(2)
        self.assertEqual(self.commands[-1], (0, 4))
        self.step(4, pressed=True)
        self.step(8)
        self.assertEqual(self.control.state, State.INITIALIZING)
        self.step(11)
        self.assertEqual(self.control.state, State.TELEOP)
        self.assertEqual([self.commands[i] for i in (0, 2, 3, 4)], list(PATH))
        self.assertEqual(self.reads, 1)

    def test_returns_from_last_command_and_resumes_with_same_origin(self):
        self.initialize()
        self.assertEqual(self.commands[-1], PATH[-1])
        self.assertEqual(self.reads, 1)
        self.step(8)
        last = (4 * 360 / 4096, -2 * 360 / 4096)
        self.assertEqual(self.commands[-1], last)

        self.sample = {1: 3, 2: 110}
        self.step(9, pressed=True)
        self.assertEqual(self.control.state, State.RETURNING)
        self.assertEqual(self.commands[-1], last)
        self.step(10, pressed=True)
        self.assertEqual(self.commands[-1], Segment(last, PATH[-1], 2).at(1))
        self.assertEqual(self.reads, 2)
        self.step(11, pressed=True)
        self.assertEqual(self.commands[-1], PATH[-1])
        self.assertEqual(self.control.state, State.PAUSED_AT_READY)
        self.step(12)
        self.assertEqual(self.commands[-2:], [Segment(last, PATH[-1], 2).at(1), PATH[-1]])
        self.assertEqual(self.reads, 2)

        self.step(13, pressed=True)
        self.assertEqual(self.control.state, State.TELEOP)
        self.assertEqual(self.commands[-1], (10 * 360 / 4096, -4 * 360 / 4096))
        self.assertEqual(self.reads, 3)

    def test_failed_publication_keeps_previous_command(self):
        self.initialize()

        def fail(_):
            raise OSError("publication failed")

        self.control.publish = fail
        with self.assertRaisesRegex(OSError, "publication failed"):
            self.step(8)
        self.assertEqual(self.control.last_command, PATH[-1])
