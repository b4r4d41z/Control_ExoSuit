"""Feetech position acquisition and the reference encoder conversion."""

from pathlib import Path
from threading import Lock

TICKS_PER_REV = 4096
DEGREES_PER_TICK = 360.0 / TICKS_PER_REV


def tick_delta(current, previous):
    if any(type(tick) is not int or not 0 <= tick < TICKS_PER_REV
           for tick in (current, previous)):
        raise ValueError("Invalid Feetech position")
    delta = current - previous
    if delta > TICKS_PER_REV / 2:
        delta -= TICKS_PER_REV
    elif delta < -TICKS_PER_REV / 2:
        delta += TICKS_PER_REV
    return delta


class EncoderAngles:
    """Accumulate signed degrees from the first sample for the entire session."""

    def __init__(self, mapping):
        self.mapping = tuple((encoder, sign) for _, encoder, sign in mapping)
        self.previous = None
        self.positions = (0.0,) * len(self.mapping)

    def update(self, sample):
        current = tuple(sample[encoder] for encoder, _ in self.mapping)
        previous = self.previous if self.previous is not None else current
        self.positions = tuple(
            angle + sign * DEGREES_PER_TICK * tick_delta(tick, last_tick)
            for angle, (_, sign), tick, last_tick in
            zip(self.positions, self.mapping, current, previous)
        )
        self.previous = current
        return self.positions


class FeetechReaders:
    """Read both encoder buses using the reference Feetech group transactions."""

    def __init__(self, left, right):
        import scservo_sdk as sdk

        if not left or not right or Path(left).resolve() == Path(right).resolve():
            raise ValueError("Set distinct left and right Feetech serial paths")
        self.sdk = sdk
        self.buses = []
        self.lock = Lock()
        try:
            for path, ids in ((left, range(1, 8)), (right, range(8, 15))):
                port = sdk.PortHandler(path)
                packet = sdk.PacketHandler(0)
                self.buses.append((path, port, packet, ids))
                if not port.openPort():
                    raise OSError(f"Could not open Feetech bus {path}")
                port.setPacketTimeoutMillis(1000)
                # The SDK configures nonblocking reads but leaves writes unbounded.
                if getattr(port, "ser", None) is not None:
                    port.ser.write_timeout = 0.1
        except Exception:
            self.close()
            raise

    def read(self):
        with self.lock:
            if not self.buses:
                raise OSError("Feetech buses are closed")
            return self._read_groups()

    def _read_groups(self):
        sample = {}
        address, size = 56, 2  # Present_Position
        for path, port, packet, ids in self.buses:
            group = self.sdk.GroupSyncRead(port, packet, address, size)
            for encoder in ids:
                group.addParam(encoder)
            result = group.txRxPacket()
            if result != self.sdk.COMM_SUCCESS:
                raise OSError(f"Feetech read failed on {path}: {packet.getTxRxResult(result)}")
            for encoder in ids:
                if not group.isAvailable(encoder, address, size):
                    raise OSError(f"Missing Feetech encoder {encoder} on {path}")
                value = group.getData(encoder, address, size)
                if type(value) is not int or not 0 <= value < TICKS_PER_REV:
                    raise ValueError(f"Invalid Feetech reading for ID {encoder} on {path}")
                sample[encoder] = value
        return sample

    def close(self):
        # A stalled USB transaction can outlast the SDK's receive timeout.
        # Leave its descriptor for process exit instead of closing during I/O.
        if not self.lock.acquire(timeout=1):
            raise TimeoutError("Feetech I/O did not finish during shutdown")
        try:
            for _, port, _, _ in self.buses:
                if port.is_open:
                    port.closePort()
            self.buses.clear()
        finally:
            self.lock.release()
