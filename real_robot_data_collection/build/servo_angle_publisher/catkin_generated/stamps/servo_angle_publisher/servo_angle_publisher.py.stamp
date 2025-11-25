#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
import scservo_sdk as scs
from sensor_msgs.msg import JointState
from joyconrobotics import JoyconRobotics  # NEW: Joy-Con control

PROTOCOL_VERSION = 0
TIMEOUT_MS = 1000
LEFT_USB_PORT = "/dev/ttyACM0"
RIGHT_USB_PORT = "/dev/ttyACM1"
MODEL_RESOLUTION = 4096
NUM_READ_RETRY = 20
NUM_WRITE_RETRY = 20

SCS_SERIES_CONTROL_TABLE = {
    "Model": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Angle_Limit": (9, 2),
    "Max_Angle_Limit": (11, 2),
    "Present_Position": (56, 2),
}

TICKS_PER_REV = 4096
TICKS_TO_DEG = 360.0 / TICKS_PER_REV
INVERTED_MOTORS = [1, 9]

class FeetechMotor:
    def __init__(self, port):
        self.port = port

    def connect(self):
        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
        if not self.port_handler.openPort():
            raise OSError(f"Failed to open port '{self.port}'.")
        self.port_handler.setPacketTimeoutMillis(TIMEOUT_MS)

    def disconnect(self):
        if self.port_handler:
            self.port_handler.closePort()
        self.packet_handler = None

    def read_positions(self, motor_ids):
        addr, bytes = SCS_SERIES_CONTROL_TABLE["Present_Position"]
        group = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, bytes)
        for idx in motor_ids:
            group.addParam(idx)
        group.txRxPacket()
        positions = [group.getData(idx, addr, bytes) for idx in motor_ids]
        return positions

def calculate_delta(current, previous):
    delta = current - previous
    if delta > TICKS_PER_REV / 2:
        delta -= TICKS_PER_REV
    elif delta < -TICKS_PER_REV / 2:
        delta += TICKS_PER_REV
    return delta

if __name__ == "__main__":
    rospy.init_node("dual_arm_angle_publisher")
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    rate = rospy.Rate(10)

    left_motor = FeetechMotor(LEFT_USB_PORT)
    right_motor = FeetechMotor(RIGHT_USB_PORT)
    left_motor.connect()
    right_motor.connect()

    left_ids = [2, 1, 3, 4, 5, 6, 7]
    right_ids = [9, 8, 10, 11, 12, 13, 14]
    joint_names = [f"j{i}" for i in range(1, 15)]

    prev_left = left_motor.read_positions(left_ids)
    prev_right = right_motor.read_positions(right_ids)
    acc_left = [0.0] * len(left_ids)
    acc_right = [0.0] * len(right_ids)

    # --- NEW: Joy-Con toggle logic ---
    jc = JoyconRobotics("right", without_rest_init=True)
    publishing = False
    last_button = 0
    last_msg = None
    rospy.loginfo("Press 'A' to start/stop publishing servo data.")
    # ---------------------------------

    try:
        while not rospy.is_shutdown():
            # NEW: toggle on button A
            state = jc.button.get_button_a()
            if state == 1 and last_button == 0:
                publishing = not publishing
                rospy.loginfo("Publishing: %s", publishing)
            last_button = state

            if publishing:
                cur_left = left_motor.read_positions(left_ids)
                cur_right = right_motor.read_positions(right_ids)

                for i, (curr, prev, mid) in enumerate(zip(cur_left, prev_left, left_ids)):
                    delta = calculate_delta(curr, prev)
                    val = delta * TICKS_TO_DEG
                    if mid in INVERTED_MOTORS:
                        val *= -1
                    acc_left[i] += val

                for i, (curr, prev, mid) in enumerate(zip(cur_right, prev_right, right_ids)):
                    delta = calculate_delta(curr, prev)
                    val = delta * TICKS_TO_DEG
                    if mid in INVERTED_MOTORS:
                        val *= -1
                    acc_right[i] += val

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = joint_names
                msg.position = acc_left + acc_right
                last_msg = msg
                pub.publish(msg)
                prev_left, prev_right = cur_left, cur_right

            else:
                # NEW: hold last state
                if last_msg:
                    pub.publish(last_msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        left_motor.disconnect()
        right_motor.disconnect()
        jc.disconnect()
