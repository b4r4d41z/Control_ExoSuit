"""Feetech position acquisition and relative encoder calibration."""

from pathlib import Path

TICKS_PER_REV = 4096
DEGREES_PER_TICK = 360.0 / TICKS_PER_REV


def tick_delta(current, previous):
    if any(type(tick) is not int or not 0 <= tick < TICKS_PER_REV
           for tick in (current, previous)):
        raise ValueError("Invalid Feetech position")
    delta = (current - previous + TICKS_PER_REV // 2) % TICKS_PER_REV - TICKS_PER_REV // 2
    if delta == -TICKS_PER_REV // 2:
        raise ValueError("Ambiguous half-turn between Feetech readings")
    return delta


class RelativeEncoders:
    def __init__(self, mapping, max_deg_s, max_age_s):
        self.mapping = tuple((encoder, sign) for _, encoder, sign in mapping)
        self.max_deg_s = max_deg_s
        self.max_age_s = max_age_s

    def calibrate(self, sample, stamp, ready):
        for encoder, _ in self.mapping:
            tick_delta(sample[encoder], sample[encoder])
        self.previous, self.stamp = dict(sample), stamp
        self.ready = tuple(ready)
        self.offset = (0.0,) * len(self.mapping)
        return self.ready

    def target(self, sample, stamp):
        elapsed = stamp - self.stamp
        if not 0 < elapsed <= self.max_age_s:
            raise ValueError("Invalid encoder sample interval")
        bound = self.max_deg_s * elapsed + DEGREES_PER_TICK
        if bound >= 180:
            raise ValueError("Encoder interval permits ambiguous wraparound")
        deltas = tuple(
            sign * DEGREES_PER_TICK * tick_delta(sample[encoder], self.previous[encoder])
            for encoder, sign in self.mapping
        )
        if any(abs(delta) > bound for delta in deltas):
            raise ValueError("Encoder velocity exceeds configured bound")
        self.offset = tuple(offset + delta for offset, delta in zip(self.offset, deltas))
        self.previous, self.stamp = dict(sample), stamp
        return tuple(ready + offset for ready, offset in zip(self.ready, self.offset))


class FeetechReaders:
    """Open each reference bus exactly once and read both sides as a set."""

    def __init__(self, left, right):
        import scservo_sdk as sdk

        if Path(left).resolve() == Path(right).resolve():
            raise ValueError("Feetech buses must have distinct serial paths")
        self.sdk = sdk
        self.packet = sdk.PacketHandler(0)
        self.ports = []
        try:
            for path, ids in ((left, range(1, 8)), (right, range(8, 15))):
                port = sdk.PortHandler(path)
                self.ports.append((port, ids))
                if not port.openPort():
                    raise OSError("Could not open Feetech bus " + path)
        except Exception:
            self.close()
            raise

    def read(self):
        sample = {}
        for port, ids in self.ports:
            for encoder in ids:
                # This SDK's group read discards each device's error byte.
                value, result, error = self.packet.read2ByteTxRx(port, encoder, 56)
                if result != self.sdk.COMM_SUCCESS or error != 0:
                    raise OSError("Missing or faulty Feetech encoder {}".format(encoder))
                if type(value) is not int or not 0 <= value < TICKS_PER_REV:
                    raise ValueError("Invalid Feetech reading for ID {}".format(encoder))
                sample[encoder] = value
        return sample

    def close(self):
        for port, _ in self.ports:
            if getattr(port, "is_open", True):
                port.closePort()
