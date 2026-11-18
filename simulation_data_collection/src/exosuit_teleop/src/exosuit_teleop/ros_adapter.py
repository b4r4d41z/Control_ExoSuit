"""ROS graph monitoring, asynchronous I/O and MoveIt state validation."""

from math import radians
from threading import Thread

from .safety import ARM, ARM_FEEDBACK, COMMAND_TYPES, GRIP, HAND_FEEDBACK, HAND_TYPE, RECORD


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
        if self.error is not None:
            raise RuntimeError(str(self.error)) from self.error
        return True, self.result


def graph_health(state, types, own_name, feedback_type):
    publishers, subscribers, _ = map(dict, state)
    types = dict(types)

    def ready(topic):
        return (types.get(topic) == COMMAND_TYPES[topic]
                and set(publishers.get(topic, ())) == {own_name}
                and bool(set(subscribers.get(topic, ())) - {own_name}))

    def source(topic, expected_type):
        names = set(publishers.get(topic, ()))
        return names if len(names) == 1 and types.get(topic) == expected_type else set()

    return (ready(ARM), ready(GRIP), ready(RECORD),
            source(ARM_FEEDBACK, feedback_type), source(HAND_FEEDBACK, HAND_TYPE))


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
