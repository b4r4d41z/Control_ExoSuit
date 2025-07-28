#!/usr/bin/env python3
import rospy
from joyconrobotics import JoyconRobotics
from dynamic_biped.msg import robot_hand_eff
from grip_control.src.kuavoSDK import kuavo

class JoyconGripControlNode:
    def __init__(self):
        rospy.init_node('joycon_grip_control_node')
        self.rate = rospy.Rate(50)

        # Connect to left and right JoyCons
        self.joy_left = JoyconRobotics("left")
        self.joy_right = JoyconRobotics("right")

        # Robot control interface
        self.robot = kuavo("3_7_kuavo")

        # Define open and closed hand positions
        self.open_left = [0, 100, 0, 0, 0, 0]
        self.close_left = [69, 99, 42, 54, 61, 60]
        self.open_right = [0, 100, 0, 0, 0, 0]
        self.close_right = [69, 99, 42, 44, 61, 60]

        # Initialize grip status
        self.prev_gripper_left = -1
        self.prev_gripper_right = -1

        # Publisher for robot_hand_eff topic
        self.publisher = rospy.Publisher('robot_hand_eff', robot_hand_eff, queue_size=10)

        self.run()

    def run(self):
        rospy.loginfo("JoyCon Grip Control node started.")
        while not rospy.is_shutdown():
            try:
                _, gripper_r, _ = self.joy_right.get_control()
                _, gripper_l, _ = self.joy_left.get_control()

                if gripper_l != self.prev_gripper_left or gripper_r != self.prev_gripper_right:
                    self.publish_hand_state(gripper_l, gripper_r)
                    self.send_to_robot(gripper_l, gripper_r)
                    self.prev_gripper_left = gripper_l
                    self.prev_gripper_right = gripper_r

            except Exception as e:
                rospy.logwarn(f"[JoyCon] Failed to get input: {e}")

            self.rate.sleep()

        self.joy_left.disconnect()
        self.joy_right.disconnect()
        rospy.loginfo("JoyCons disconnected.")

    def publish_hand_state(self, gripper_l, gripper_r):
        msg = robot_hand_eff()
        msg.header.stamp = rospy.Time.now()

        left_hand = self.close_left if gripper_l == 1 else self.open_left
        right_hand = self.close_right if gripper_r == 1 else self.open_right

        msg.data = left_hand + right_hand
        self.publisher.publish(msg)
        rospy.loginfo(f"Published hand state: L={'closed' if gripper_l==1 else 'open'}, R={'closed' if gripper_r==1 else 'open'}")

    def send_to_robot(self, gripper_l, gripper_r):
        l = self.close_left if gripper_l == 1 else self.open_left
        r = self.close_right if gripper_r == 1 else self.open_right
        self.robot.set_robot_end_position_control(l, r)

if __name__ == '__main__':
    try:
        JoyconGripControlNode()
    except rospy.ROSInterruptException:
        pass
