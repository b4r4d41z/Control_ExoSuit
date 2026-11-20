import math
import sys
import unittest
from unittest.mock import patch
from threading import Event
from types import SimpleNamespace

from exosuit_teleop.encoder_reader import FeetechReaders, RelativeEncoders, tick_delta
from exosuit_teleop.joycon_input import JoyCon
from exosuit_teleop.ros_adapter import arm_graph
from exosuit_teleop.safety import (
    ARM, ARM_FEEDBACK, COMMAND_TYPES, Feedback, Safety, decode_feedback, fresh,
)
from exosuit_teleop.state_machine import ARM_PERIOD, State, Teleoperation
from exosuit_teleop.trajectory import Limits, Segment, named_path
from exosuit_teleop.trajectory_rejoin import PathPlanner, candidates


class EncodersTest(unittest.TestCase):
    def test_wrap_calibration_mapping_and_speed(self):
        self.assertEqual(tick_delta(1, 4095), 2)
        self.assertEqual(tick_delta(4095, 1), -2)
        for tick in (-1, 4096, 2048):
            with self.assertRaises(ValueError):
                tick_delta(tick, 0)

        encoder = RelativeEncoders([("r1", 2, 1), ("r2", 1, -1)], 100, 0.2)
        self.assertEqual(encoder.calibrate({1: 4095, 2: 4}, 1.0, (21, 14)), (21, 14))
        self.assertEqual(encoder.target({1: 1, 2: 7}, 1.1),
                         (21 + 3 * 360 / 4096, 14 - 2 * 360 / 4096))
        with self.assertRaises(ValueError):
            encoder.target({1: 90, 2: 7}, 1.11)
        with self.assertRaises(ValueError):
            encoder.target({1: 1, 2: 7}, 2.0)

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


LIMITS = Limits((-20.0, -20.0), (20.0, 20.0), (30.0, 25.0), (80.0, 100.0))
PATH = ((0.0, 0.0), (0.0, 8.0), (10.0, 8.0), (10.0, 10.0))


class MovementTest(unittest.TestCase):
    def test_pose_validation(self):
        self.assertEqual(named_path(("a", "b"), ("ZERO", "ELBOW", "READY"),
                                    {"ZERO": {}, "ELBOW": {"b": 8}, "READY": {"a": 9}})[-1],
                         (9, 8))
        for bad in ((21, 0), (0,), (0, float("nan"))):
            with self.assertRaises(ValueError):
                Segment.between(PATH[0], bad, LIMITS, ARM_PERIOD)

    def test_quintic_is_smooth_and_bounded(self):
        move = Segment.between((-10.0, 0.0), (10.0, 15.0), LIMITS, ARM_PERIOD)
        self.assertEqual(move.at(-1), move.start)
        self.assertEqual(move.at(move.duration + 1), move.goal)
        step = move.duration / 1000
        positions = [move.at(i * step) for i in range(1001)]
        for index, (vmax, amax) in enumerate(zip(LIMITS.velocity, LIMITS.acceleration)):
            velocities = [(b[index] - a[index]) / step for a, b in zip(positions, positions[1:])]
            self.assertLess(max(map(abs, velocities)), vmax + 1e-6)
            self.assertLess(max(abs(b - a) / step for a, b in zip(velocities, velocities[1:])), amax + 1e-6)
            self.assertLess(abs(velocities[0]), 1e-3)
            self.assertLess(abs(velocities[-1]), 1e-3)

    def test_rejoin_rejects_nearest_collision_and_uses_suffix(self):
        start = (3.0, 2.0)
        closest = candidates(start, PATH)[0][1]
        self.assertEqual(closest, (0.0, 2.0))
        self.assertNotIn(PATH[0], [point for _, point, _ in candidates(start, PATH)])

        def valid(pose):
            return not (1.0 <= pose[0] <= 2.0 and 1.8 <= pose[1] <= 2.2)

        planner = PathPlanner(LIMITS, 0.1, ARM_PERIOD, valid)
        route = planner.plan("rejoin", start, PATH, Event())
        self.assertEqual(route[0].start, start)
        self.assertNotEqual(route[0].goal, closest)
        self.assertEqual(route[-1].goal, PATH[-1])
        self.assertTrue(all(a.goal == b.start for a, b in zip(route, route[1:])))

    def test_unreachable_and_cancelled_plan(self):
        planner = PathPlanner(LIMITS, 0.2, ARM_PERIOD, lambda pose: pose[1] < 9)
        self.assertIsNone(planner.plan("rejoin", (3.0, 2.0), PATH, Event()))
        self.assertIsNone(planner.plan("initial", PATH[0], PATH, Event()))
        self.assertFalse(planner.plan("live", PATH[2], (PATH[-1],), Event()))
        cancelled = Event()
        cancelled.set()
        self.assertIsNone(planner.plan("rejoin", PATH[2], PATH, cancelled))


class ImmediateJob:
    def __init__(self, planner):
        self.planner = planner
        self.last_kind = None

    def start(self, *args):
        self.last_kind = args[0]
        self.result = self.planner(*args)
        return True

    def poll(self):
        return True, self.result

    def discard(self):
        self.result = None


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.position = PATH[0]
        self.settings = SimpleNamespace(arrival=0.01, tracking=5.0, stopped=0.01, age=0.3,
                                        stop_settle=0.3, encoder_age=0.4, plan_timeout=2.0)
        planner = PathPlanner(LIMITS, 0.2, ARM_PERIOD, lambda _: True)
        self.job = ImmediateJob(planner.plan)
        encoders = RelativeEncoders([("a", 1, 1), ("b", 2, 1)], 100.0, 0.4)
        self.control = Teleoperation(PATH, self.settings, LIMITS, encoders, self.job)

    def step(self, *, a=False, sample=None, feedback=True, follow=True):
        self.now += 0.11
        fb = Feedback(self.position, (0, 0), self.now, self.now, "source") if feedback else None
        result = self.control.step(self.now, fb, a, (sample, self.now) if sample else None)
        if result is not None and follow:
            self.position = result
        return result

    def until(self, state):
        for _ in range(120):
            if self.control.state == state:
                return
            self.step()
        self.fail("Timed out at {} {}".format(self.control.state, self.control.reason))

    def test_initial_calibration_hold_and_return(self):
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.INITIALIZING)
        self.until(State.CALIBRATING)
        self.assertEqual(self.position, PATH[-1])
        self.assertEqual(self.step(sample={1: 4095, 2: 1}), PATH[-1])
        self.assertEqual(self.control.state, State.TELEOP)
        self.assertAlmostEqual(self.step(sample={1: 0, 2: 1})[0], PATH[-1][0] + 360 / 4096)
        self.assertEqual(self.job.last_kind, "initial")
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.HOLD)
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.HOLD)
        self.step()
        self.position = (3.0, 2.0)  # Actual controller stop pose can differ from last target.
        self.step(a=True)
        self.assertEqual(self.control.state, State.RETURNING)
        self.assertEqual(self.step(), (3.0, 2.0))
        self.until(State.CALIBRATING)
        self.assertEqual(self.step(sample={1: 100, 2: 200}), PATH[-1])
        self.assertEqual(self.control.state, State.TELEOP)

    def test_live_tracks_latest_sample_without_a_queued_move(self):
        self.step(a=True)
        self.until(State.CALIBRATING)
        self.step(sample={1: 100, 2: 100})
        first = self.step(sample={1: 210, 2: 100})
        self.assertAlmostEqual(first[0], PATH[-1][0] + LIMITS.velocity[0] * ARM_PERIOD)
        reversed_target = self.step(sample={1: 100, 2: 100})
        self.assertEqual(reversed_target, PATH[-1])

    def test_pending_collision_step_is_discarded_when_target_reverses(self):
        self.control.check_live = True
        self.step(a=True)
        self.until(State.CALIBRATING)
        self.step(sample={1: 100, 2: 100})
        self.assertEqual(self.step(sample={1: 110, 2: 100}), PATH[-1])
        self.assertEqual(self.job.last_kind, "live")
        self.assertEqual(self.step(sample={1: 90, 2: 100}), PATH[-1])
        self.assertAlmostEqual(self.step(sample={1: 90, 2: 100})[0],
                               PATH[-1][0] - 10 * 360 / 4096)

    def test_requires_zero_and_fresh_feedback_latches_fault(self):
        self.position = (0.0, 2.0)
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.IDLE)
        self.assertIsNone(self.step(a=True, feedback=False))
        self.position = PATH[0]
        self.step(a=True)
        self.assertIsNone(self.step(feedback=False))
        self.assertEqual(self.control.state, State.FAULT)
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.FAULT)

    def test_abort_and_no_return_path_keep_hold(self):
        self.step(a=True)
        self.assertIsNone(self.step(a=True))
        self.assertEqual(self.control.state, State.HOLD)
        self.step()
        self.step()
        self.job.planner = lambda *_: None
        self.step(a=True)
        self.assertEqual(self.control.state, State.RETURNING)
        self.assertIsNone(self.step())
        self.assertEqual(self.control.state, State.HOLD)

    def test_does_not_advance_without_arrival(self):
        self.step(a=True)
        for _ in range(120):
            if self.control.state == State.FAULT:
                break
            self.step(follow=self.control.command is None or self.control.command[1] < 5)
        self.assertEqual(self.control.state, State.FAULT)
        self.assertNotEqual(self.position, PATH[-1])


class SafetyTest(unittest.TestCase):
    def test_default_profile_disables_movement(self):
        with self.assertRaises(ValueError):
            Safety.load({}, ("a",))

    def test_simulator_profile_can_run_without_moveit(self):
        settings = {
            "verified_watchdog": True, "feedback_type": "sensor_msgs/JointState",
            "feedback_units": "radians", "feedback_age_s": 0.3, "tracking_deg": 2,
            "arrival_deg": 0.1, "stopped_deg_s": 0.5, "stop_settle_s": 0.5,
            "encoder_max_deg_s": 100, "encoder_age_s": 0.4,
        }
        self.assertEqual(Safety.load(settings, ("a",)).service, "")
        settings["validity_service"] = "/check_state_validity"
        with self.assertRaisesRegex(ValueError, "approved scene"):
            Safety.load(settings, ("a",))

    def test_decode_freshness_and_source(self):
        mock = SimpleNamespace(name=["b", "a"], position=[math.pi, math.pi / 2], velocity=[0, 0.1],
                               header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 5)),
                               _connection_header={"callerid": "/bridge"})
        fb = decode_feedback(mock, (), "radians", ("a", "b"), 10)
        self.assertEqual(fb.position, (90, 180))
        self.assertTrue(fresh(fb, 10.1, 5.1, {"/bridge"}, 0.2))
        self.assertFalse(fresh(fb, 10.3, 5.1, {"/bridge"}, 0.2))
        self.assertFalse(fresh(fb, 10.1, 5.3, {"/bridge"}, 0.2))
        self.assertFalse(fresh(fb, 10.1, 5.1, {"/bridge", "/unexpected"}, 0.2))
        self.assertFalse(fresh(None, 10.1, 5.1, {"/bridge"}, 0.2))
        mock.velocity.pop()
        with self.assertRaises(ValueError):
            decode_feedback(mock, (), "radians", ("a", "b"), 10)

    def test_graph_rejects_competitor_and_feedback_type_mismatch(self):
        pubs = [(topic, ["/exo"]) for topic in COMMAND_TYPES]
        pubs.append((ARM_FEEDBACK, ["/bridge"]))
        subs = [(topic, ["/bridge"]) for topic in COMMAND_TYPES]
        types = list(COMMAND_TYPES.items()) + [(ARM_FEEDBACK, "sensor_msgs/JointState")]
        ok = arm_graph((pubs, subs, []), types, "/exo", "sensor_msgs/JointState")
        self.assertEqual(ok, (True, {"/bridge"}))
        pubs[0] = (ARM, ["/exo", "/old"])
        wrong = arm_graph((pubs, subs, []), types, "/exo", "dynamic_biped/robotQVTau")
        self.assertEqual(wrong, (False, set()))
