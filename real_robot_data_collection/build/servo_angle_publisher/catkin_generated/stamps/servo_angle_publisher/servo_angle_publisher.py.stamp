#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
import scservo_sdk as scs
from sensor_msgs.msg import JointState
from joyconrobotics import JoyconRobotics  # add Joy-Con


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
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Offset": (31, 2),
    "Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Speed_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Speed": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Speed": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
    "Maximum_Acceleration": (85, 2),
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

        comm = group.txRxPacket()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Read failed: {self.packet_handler.getTxRxResult(comm)}"
            )

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

    left_motor_comm = FeetechMotor(LEFT_USB_PORT)
    right_motor_comm = FeetechMotor(RIGHT_USB_PORT)

    left_motor_comm.connect()
    right_motor_comm.connect()

    left_motor_ids = [1, 2, 3, 4, 5, 6, 7]
    right_motor_ids = [8, 9, 10, 11, 12, 13, 14]

    joint_names = [
        "j1", "j2", "j3", "j4",
        "j5", "j6", "j7",
        "j8", "j9", "j10", "j11",
        "j12", "j13", "j14"
    ]

    previous_positions_left = left_motor_comm.read_positions(left_motor_ids)
    previous_positions_right = right_motor_comm.read_positions(right_motor_ids)
    accumulated_degrees_left = [0.0] * len(left_motor_ids)
    accumulated_degrees_right = [0.0] * len(right_motor_ids)

    # ============== index mapping  =============
    left_index_map = [1, 0, 2, 3, 4, 5, 6]     # swap j1 and j2
    right_index_map = [1, 0, 2, 3, 4, 5, 6]    # swap j8 and j9
    # ===========================================

    jc = JoyconRobotics("right", without_rest_init=True)
    publishing = False
    last_button = 0

    rospy.loginfo("Press 'A' to start/stop publishing servo angle data.")

    try:
        while not rospy.is_shutdown():
            state = jc.button.get_button_a()
            if state == 1 and last_button == 0:
                publishing = not publishing
                rospy.loginfo("Publishing: %s", publishing)
            last_button = state

            if publishing:
                current_positions_left = left_motor_comm.read_positions(left_motor_ids)
                current_positions_right = right_motor_comm.read_positions(right_motor_ids)

                for i, (curr, prev, motor_id) in enumerate(
                    zip(current_positions_left, previous_positions_left, left_motor_ids)
                ):
                    delta = calculate_delta(curr, prev)
                    value = delta * TICKS_TO_DEG
                    if motor_id in INVERTED_MOTORS:
                        value *= -1
                    accumulated_degrees_left[i] += value

                for i, (curr, prev, motor_id) in enumerate(
                    zip(current_positions_right, previous_positions_right, right_motor_ids)
                ):
                    delta = calculate_delta(curr, prev)
                    value = delta * TICKS_TO_DEG
                    if motor_id in INVERTED_MOTORS:
                        value *= -1
                    accumulated_degrees_right[i] += value

                # === apply mapping before publishing ===
                logical_left = [accumulated_degrees_left[i] for i in left_index_map]
                logical_right = [accumulated_degrees_right[i] for i in right_index_map]
                # =======================================

                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = joint_names
                joint_msg.position = logical_left + logical_right
                pub.publish(joint_msg)

                previous_positions_left = current_positions_left
                previous_positions_right = current_positions_right

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        left_motor_comm.disconnect()
        right_motor_comm.disconnect()
        jc.disconnect()
