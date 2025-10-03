#!/usr/bin/env python
import rospy
from dynamic_biped.msg import robot_hand_eff
from joyconrobotics import JoyconRobotics

# Presets for hand states
open_left_hand = [0,100,0,0,0,0]
close_left_hand = [69,99,42,54,61,60]

open_right_hand = [0,100,0,0,0,0]
close_right_hand = [69,99,42,44,61,60]

class JoyconHandPublisher:
    def __init__(self):
        rospy.init_node('joycon_hand_publisher', anonymous=True)
        self.pub = rospy.Publisher('robot_hand_eff', robot_hand_eff, queue_size=10)
        self.rate = rospy.Rate(50)  # Hz

        # Init Joy-Cons
        self.joycon_right = JoyconRobotics("right")
        self.joycon_left = JoyconRobotics("left")

        rospy.loginfo("Joycon hand publisher started (independent control for each hand)")

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            # Get data from Joy-Cons
            _, gripper_r, _ = self.joycon_right.get_control()
            _, gripper_l, _ = self.joycon_left.get_control()

            # Decide left hand
            if gripper_l == 1:
                left = close_left_hand
            else:
                left = open_left_hand

            # Decide right hand
            if gripper_r == 1:
                right = close_right_hand
            else:
                right = open_right_hand

            # Build ROS message
            msg = robot_hand_eff()
            msg.header.stamp = rospy.Time.now()
            msg.data = left + right   # combine two arrays into one

            # Publish
            self.pub.publish(msg)
            self.rate.sleep()

        # On shutdown: disconnect Joy-Cons
        self.joycon_right.disconnect()
        self.joycon_left.disconnect()

if __name__ == "__main__":
    try:
        JoyconHandPublisher()
    except rospy.ROSInterruptException:
        pass
