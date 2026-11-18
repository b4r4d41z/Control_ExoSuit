"""Deployment prerequisites and input freshness checks."""

from dataclasses import dataclass
from math import degrees, isfinite
from typing import Optional

from .trajectory import positive

ARM = "/kuavo_arm_traj"
GRIP = "/control_robot_hand_position"
ARM_FEEDBACK = "/robot_arm_q_v_tau"
HAND_FEEDBACK = "/robot_hand_position"
RECORD = "/data_collection/recording"
HAND_TYPE = "dynamic_biped/robotHandPosition"
COMMAND_TYPES = {ARM: "sensor_msgs/JointState", GRIP: HAND_TYPE, RECORD: "std_msgs/Bool"}


@dataclass(frozen=True)
class Safety:
    service: str
    spacing: float
    plan_timeout: float
    age: float
    tracking: float
    arrival: float
    stopped: float
    stop_settle: float
    encoder_speed: float
    encoder_age: float
    feedback_type: str
    feedback_units: str
    feedback_order: tuple

    @classmethod
    def load(cls, config, names):
        scene = config.get("approved_scene")
        if not isinstance(scene, str) or not scene.strip() or config.get("verified_watchdog") is not True:
            raise ValueError("An approved scene and a verified controller stop/hold are required")
        if config.get("verified_bridge") is not True:
            raise ValueError("The bridge and recording command contract require verification")
        service = config.get("validity_service")
        if not isinstance(service, str) or not service.startswith("/") or service == "/":
            raise ValueError("Configure an absolute MoveIt state validity service name")
        fields = ("collision_sample_deg", "plan_timeout_s", "feedback_age_s",
                  "tracking_deg", "arrival_deg", "stopped_deg_s", "stop_settle_s",
                  "encoder_max_deg_s", "encoder_age_s")
        spacing, timeout, age, tracking, arrival, stopped, settle, speed, encoder_age = (
            positive(config.get(field), field) for field in fields
        )
        if arrival > tracking or speed * encoder_age + 360.0 / 4096 >= 180:
            raise ValueError("Arrival or encoder bounds are inconsistent")
        kind = config.get("feedback_type")
        units = config.get("feedback_units")
        order = tuple(config.get("feedback_order", ()))
        if kind == "dynamic_biped/robotQVTau":
            if config.get("verified_unstamped_feedback") is not True:
                raise ValueError("robotQVTau requires a verified live, non-replaying feedback source")
            if len(order) != len(names) or set(order) != set(names):
                raise ValueError("Confirm the robotQVTau wire order for this bridge")
        elif kind != "sensor_msgs/JointState":
            raise ValueError("Unsupported arm feedback type")
        if units not in ("degrees", "radians"):
            raise ValueError("Confirm arm feedback units")
        return cls(service, spacing, timeout, age, tracking, arrival, stopped, settle,
                   speed, encoder_age, kind, units, order)


@dataclass(frozen=True)
class Feedback:
    position: tuple
    velocity: tuple
    received: float
    stamp: Optional[float]
    source: str

    def stopped(self, tolerance):
        return all(abs(value) <= tolerance for value in self.velocity)


def decode_feedback(message, order, units, wire_names, received):
    if order:
        names, positions, velocities = order, message.q, message.v
    else:
        names, positions, velocities = message.name, message.position, message.velocity
    if len(names) != len(set(names)) or len(positions) != len(names) or len(velocities) != len(names):
        raise ValueError("Invalid feedback arrays")
    indices = {name: i for i, name in enumerate(names)}

    def ordered(values):
        result = tuple(values[indices[name]] for name in wire_names)
        if not all(isfinite(value) for value in result):
            raise ValueError("Non-finite joint feedback")
        return tuple(map(degrees, result)) if units == "radians" else result

    stamp = None if order else message.header.stamp.to_sec()
    source = (message._connection_header or {}).get("callerid")
    if not source or (stamp is not None and (not isfinite(stamp) or stamp <= 0)):
        raise ValueError("Joint feedback needs a source and a valid timestamp when stamped")
    return Feedback(ordered(positions), ordered(velocities), received, stamp, source)


def fresh(sample, now, ros_now, sources, age):
    return (sample is not None and set(sources) == {sample.source}
            and 0 <= now - sample.received <= age
            and (sample.stamp is None or 0 <= ros_now - sample.stamp <= age))
