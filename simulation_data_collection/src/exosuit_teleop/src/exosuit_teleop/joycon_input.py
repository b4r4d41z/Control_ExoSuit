"""Use the same HID-backed JoyconRobotics instances as the reference nodes."""

from contextlib import ExitStack
from fcntl import LOCK_EX, flock
from pathlib import Path
from time import monotonic

from rospkg import get_ros_home


class JoyCon:
    def __init__(self, side):
        self.side = side
        try:
            from joyconrobotics import JoyconRobotics

            # Its HID setup waits for uncorrelated replies on the same controller.
            # Let each process finish that setup before another opens this side.
            lock_dir = Path(get_ros_home())
            lock_dir.mkdir(parents=True, exist_ok=True)
            with (lock_dir / f"exosuit-joycon-{side}.lock").open("a") as lock:
                flock(lock, LOCK_EX)
                self.device = JoyconRobotics(side, without_rest_init=True)
        except Exception as error:
            raise RuntimeError(
                f"Could not open {side} Joy-Con via JoyconRobotics. "
                "Check pairing, supported model, and HID access in this container."
            ) from error
        self.button = self.device.button
        self.report = None
        self.last_report = monotonic()

    def check(self):
        # The pinned library caches the last report even after its reader exits.
        # Every successful read assigns a new report object, even with unchanged keys.
        now = monotonic()
        report = self.button._input_report
        if report is not self.report:
            self.report, self.last_report = report, now
        reader = self.button._update_input_report_thread
        if not reader.is_alive() or now - self.last_report > 0.5:
            raise OSError(f"{self.side} Joy-Con stopped delivering HID reports")

    def get_control(self):
        return self.device.get_control()

    def close(self):
        self.device.running = False
        disconnect = getattr(self.device, "disconnect", None) or self.device.disconnnect
        # The pinned disconnect method closes only the primary handle.
        with ExitStack() as cleanup:
            cleanup.callback(self.device.gyro._close)
            cleanup.callback(self.device.button._close)
            disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
