"""Exclusive HID-Nintendo evdev input for the two Joy-Cons."""

from pathlib import Path


class ButtonEdges:
    def __init__(self, debounce_s=0.25):
        self.previous = None
        self.last = {}
        self.debounce = debounce_s

    def read(self, pressed, now):
        pressed = set(pressed)
        rising = pressed - self.previous if self.previous is not None else set()
        self.previous = pressed
        accepted = {key for key in rising if now - self.last.get(key, -float("inf")) >= self.debounce}
        self.last.update((key, now) for key in accepted)
        return accepted


class JoyCons:
    def __init__(self, left, right):
        from evdev import InputDevice, ecodes

        if not left or not right or Path(left).resolve() == Path(right).resolve():
            raise ValueError("Set distinct left and right Joy-Con event paths")
        self.devices = []
        try:
            for path, names in (
                (left, {"ZL": ecodes.BTN_TL2}),
                (right, {"A": ecodes.BTN_EAST, "X": ecodes.BTN_NORTH,
                         "Y": ecodes.BTN_WEST, "ZR": ecodes.BTN_TR2}),
            ):
                device = InputDevice(path)
                self.devices.append((device, names))
                if device.info.vendor != 0x057E or not set(names.values()).issubset(
                    device.capabilities().get(ecodes.EV_KEY, [])
                ):
                    raise ValueError("Unexpected Joy-Con identity or key mapping: " + path)
                device.grab()
        except Exception:
            self.close()
            raise

    def read(self):
        pressed = set()
        for device, names in self.devices:
            # Drain the fd and use the kernel's current state (also handles SYN_DROPPED).
            # Surface disconnects rather than continuing to use a cached button mask.
            try:
                for _ in device.read():
                    pass
            except BlockingIOError:
                pass
            active = device.active_keys()
            pressed.update(name for name, code in names.items() if code in active)
        return pressed

    def close(self):
        for device, _ in self.devices:
            device.close()
