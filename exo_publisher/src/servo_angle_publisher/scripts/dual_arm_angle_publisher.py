#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
import scservo_sdk as scs
from sensor_msgs.msg import JointState


PROTOCOL_VERSION = 0
TIMEOUT_MS = 1000
LEFT_USB_PORT = "/dev/ttyACM1"
RIGHT_USB_PORT = "/dev/ttyACM0"
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
TICKS_TO_RAD = 2 * math.pi / TICKS_PER_REV

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
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10) # /joint_states #/kuavo_arm_traj
    rate = rospy.Rate(10)

    left_motor_comm = FeetechMotor(LEFT_USB_PORT)
    right_motor_comm = FeetechMotor(RIGHT_USB_PORT) 
    
    left_motor_comm.connect()
    right_motor_comm.connect()

    left_motor_ids = [1, 2, 3, 4, 5, 6, 7]
    right_motor_ids = [8, 9, 10, 11, 12, 13, 14]

    joint_names = [
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
        "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
        "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"
    ]

    previous_positions_left = left_motor_comm.read_positions(left_motor_ids)
    previous_positions_right = right_motor_comm.read_positions(right_motor_ids)
    accumulated_radians_left = [0.0] * len(left_motor_ids)
    accumulated_radians_right = [0.0] * len(right_motor_ids)

    try:
        while not rospy.is_shutdown():
            current_positions_left = left_motor_comm.read_positions(left_motor_ids)
            current_positions_right = right_motor_comm.read_positions(right_motor_ids)

            for i, (curr, prev) in enumerate(zip(current_positions_left, previous_positions_left)):
                delta = calculate_delta(curr, prev)
                accumulated_radians_left[i] += delta * TICKS_TO_RAD

            for i, (curr, prev) in enumerate(zip(current_positions_right, previous_positions_right)):
                delta = calculate_delta(curr, prev)
                accumulated_radians_right[i] += delta * TICKS_TO_RAD

            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = joint_names
            joint_msg.position = accumulated_radians_left + accumulated_radians_right
            pub.publish(joint_msg)

            previous_positions_left = current_positions_left
            previous_positions_right = current_positions_right
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        left_motor_comm.disconnect()
        right_motor_comm.disconnect()
