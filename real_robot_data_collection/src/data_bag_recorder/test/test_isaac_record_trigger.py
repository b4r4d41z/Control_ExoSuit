"""Hardware-free checks: python3 -m unittest discover -s test -v."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class ROSException(Exception):
    pass


class ROSInterruptException(Exception):
    pass


class RecordingTriggerTest(unittest.TestCase):
    def setUp(self):
        self.publisher = Mock()
        self.rospy = Mock(
            ROSException=ROSException, ROSInterruptException=ROSInterruptException
        )
        self.rospy.Publisher.return_value = self.publisher
        self.joycon = SimpleNamespace(button=Mock(), disconnect=Mock())
        self.connect = Mock(return_value=self.joycon)
        self.Bool = lambda data: SimpleNamespace(data=data)
        modules = {
            "rospy": self.rospy,
            "joyconrobotics": SimpleNamespace(JoyconRobotics=self.connect),
            "std_msgs": SimpleNamespace(),
            "std_msgs.msg": SimpleNamespace(Bool=self.Bool),
        }
        script = Path(__file__).resolve().parents[1] / "scripts/isaac_record_trigger.py"
        spec = importlib.util.spec_from_file_location("isaac_record_trigger", script)
        self.node = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.node)

    def run_buttons(self, states):
        self.rospy.is_shutdown.side_effect = [False] * len(states) + [True]
        self.joycon.button.get_button_x.side_effect = states
        self.node.main()

    def published(self):
        return [call.args[0].data for call in self.publisher.publish.call_args_list]

    def test_repeated_presses_toggle_but_holding_and_release_do_not(self):
        self.run_buttons([0, 1, 1, 1, 0, 0, 1, 1, 0, 1])
        self.assertEqual(self.published(), [False, True, False, True, False])
        self.connect.assert_called_once_with("right", without_rest_init=True)
        self.rospy.Publisher.assert_called_once_with(
            "/data_collection/recording", self.Bool, queue_size=1, latch=True
        )
        self.rospy.Rate.assert_called_once_with(20)
        self.joycon.disconnect.assert_called_once_with()

    def test_no_presses_stays_off(self):
        self.run_buttons([0, 0, 0])
        self.assertEqual(self.published(), [False, False])

    def test_pinned_library_disconnect_spelling(self):
        del self.joycon.disconnect
        self.joycon.disconnnect = Mock()
        self.run_buttons([1])
        self.assertEqual(self.published(), [False, True, False])
        self.joycon.disconnnect.assert_called_once_with()

    def test_shutdown_hook_and_interrupted_sleep_clean_up_once(self):
        def shutdown():
            self.rospy.on_shutdown.call_args.args[0]()
            raise ROSInterruptException()

        self.rospy.Rate.return_value.sleep.side_effect = shutdown
        self.run_buttons([1])
        self.assertEqual(self.published(), [False, True, False])
        self.joycon.disconnect.assert_called_once_with()

    def test_final_publish_failure_still_disconnects(self):
        self.publisher.publish.side_effect = [None, None, ROSException("closed")]
        self.run_buttons([1])
        self.joycon.disconnect.assert_called_once_with()
        self.rospy.logwarn.assert_called_once()

    def test_connection_failure_leaves_recording_off(self):
        self.connect.side_effect = OSError("controller absent")
        with self.assertRaises(OSError):
            self.node.main()
        self.assertEqual(self.published(), [False, False])
        self.joycon.disconnect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
