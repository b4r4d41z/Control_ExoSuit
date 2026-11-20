"""ROS graph monitoring, asynchronous I/O and MoveIt state validation."""

from ipaddress import IPv4Address
from math import radians
import os
from socket import AF_INET, SOCK_STREAM, socket
from urllib.parse import urlsplit
from threading import Thread

from .safety import ARM, ARM_FEEDBACK, COMMAND_TYPES


class Job:
    """One daemon operation at a time. An unresponsive I/O call cannot block stopping."""

    def __init__(self, operation):
        self.operation = operation
        self.thread = None
        self.discarded = False

    def discard(self):
        self.discarded = True

    def start(self, *args):
        if self.thread is not None and (self.thread.is_alive() or not self.discarded):
            return False
        self.discarded = False
        self.result, self.error = None, None

        def run():
            try:
                self.result = self.operation(*args)
            except Exception as error:
                self.error = error

        self.thread = Thread(target=run, daemon=True)
        self.thread.start()
        return True

    def poll(self):
        if self.thread is None or self.thread.is_alive():
            return False, None
        self.thread = None
        if self.discarded:
            return False, None
        if self.error is not None:
            raise RuntimeError(str(self.error)) from self.error
        return True, self.result


def require_simulation_master():
    """Check the selected graph before constructing any command publisher."""
    import rosgraph
    import rospy

    expected = rospy.get_param("~sim_master_uri")
    actual = rosgraph.get_master_uri()
    if rospy.get_param("~simulation_only") is not True:
        raise ValueError("Confirm the isolated simulation network with simulation_only:=true")
    if expected != actual:
        raise ValueError(f"Select the dedicated simulation master {expected}, current master is {actual}")
    parsed = urlsplit(actual)
    host = IPv4Address(parsed.hostname)
    if (parsed.scheme != "http" or parsed.netloc != f"{host}:{parsed.port}"
            or parsed.path or parsed.query or parsed.fragment
            or parsed.port is None or parsed.port in (0, 11311)
            or host.is_unspecified or host.is_multicast):
        raise ValueError("Use a local IPv4 simulation master on a dedicated port (default 11321)")
    if os.environ.get("ROS_HOSTNAME") or os.environ.get("ROS_IP") != str(host):
        raise ValueError(f"Unset ROS_HOSTNAME and set ROS_IP={host}")
    with socket(AF_INET, SOCK_STREAM) as probe:
        probe.bind((str(host), 0))  # Reject a remote master even if its URI was supplied explicitly.


def arm_graph(state, types, own_name, feedback_type):
    publishers, subscribers, _ = map(dict, state)
    types = dict(types)
    sources = set(publishers.get(ARM_FEEDBACK, ()))
    if len(sources) != 1 or types.get(ARM_FEEDBACK) != feedback_type:
        sources = set()
    ready = (types.get(ARM) == COMMAND_TYPES[ARM]
             and set(publishers.get(ARM, ())) == {own_name}
             and bool(set(subscribers.get(ARM, ())) - {own_name}))
    return ready, sources


class MoveItValidity:
    def __init__(self, service, names):
        import rospy
        from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

        self.proxy = rospy.ServiceProxy(service, GetStateValidity, persistent=True)
        self.request_type = GetStateValidityRequest
        self.names = list(names)

    def __call__(self, pose):
        request = self.request_type()
        # Use the planning scene's other joints and the entire robot collision
        # model (leave group_name empty). The scene must contain the table.
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = self.names
        request.robot_state.joint_state.position = list(map(radians, pose))
        return bool(self.proxy(request).valid)
