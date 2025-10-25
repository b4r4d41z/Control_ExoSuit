#!/usr/bin/env python3
import rospy
from dynamic_biped.msg import robotHandPosition
from joyconrobotics import JoyconRobotics

# Presets for hand states
open_left_hand = [0, 100, 0, 0, 0, 0]
close_left_hand = [69, 99, 42, 54, 61, 60]

open_right_hand = [0, 100, 0, 0, 0, 0]
close_right_hand = [69, 99, 42, 44, 61, 60]


class JoyconHandPublisher:
    def __init__(self):
        rospy.init_node('joycon_hand_publisher', anonymous=True)
        
        # Publisher to the correct topic
        self.pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz
        
        # Initialize Joy-Cons
        self.joycon_right = JoyconRobotics("right")
        self.joycon_left = JoyconRobotics("left")
        
        rospy.loginfo("Joycon hand publisher started → publishing to /control_robot_hand_position")
        
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            # Get button data from both Joy-Cons
            _, gripper_r, _ = self.joycon_right.get_control()
            _, gripper_l, _ = self.joycon_left.get_control()

            # Determine left hand state
            left = close_left_hand if gripper_l == 1 else open_left_hand

            # Determine right hand state
            right = close_right_hand if gripper_r == 1 else open_right_hand

            # Build ROS message of type robotHandPosition
            msg = robotHandPosition()
            msg.header.stamp = rospy.Time.now()
            msg.left_hand_position = left
            msg.right_hand_position = right

            # Publish message to the topic
            self.pub.publish(msg)

            # Wait for the next loop
            self.rate.sleep()

        # On shutdown: disconnect Joy-Cons
        self.joycon_right.disconnect()
        self.joycon_left.disconnect()


if __name__ == "__main__":
    try:
        JoyconHandPublisher()
    except rospy.ROSInterruptException:
        pass
