#!/usr/bin/env python3
import rospy
from dynamic_biped.msg import robotHandPosition
from joyconrobotics import JoyconRobotics
import time

# Presets for hand states
open_left_hand = [0, 100, 0, 0, 0, 0]
close_left_hand = [69, 99, 42, 54, 61, 60]

open_right_hand = [0, 100, 0, 0, 0, 0]
close_right_hand = [69, 99, 42, 44, 61, 60]


class GripStatePublisher:
    def __init__(self):
        rospy.init_node('grip_state_publisher', anonymous=True)
        self.pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz
        
        # Initialize Joy-Con controller (right side)
        self.jc = JoyconRobotics("right", without_rest_init=True)
        
        rospy.loginfo("GripStatePublisher started → press 'Y' to start/stop publishing")
        
        self.publishing = False
        self.last_button_state = 0
        self.last_msg = None  # to store the last published message

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            # Read button Y state from Joy-Con
            button_state = self.jc.button.get_button_y()

            # Toggle publishing mode on button press
            if button_state == 1 and self.last_button_state == 0:
                self.publishing = not self.publishing
                if self.publishing:
                    rospy.loginfo("Publishing started.")
                else:
                    rospy.loginfo("Publishing stopped. Holding last state.")
            
            self.last_button_state = button_state

            # Publish hand positions only when active
            if self.publishing:
                _, gripper_right, _ = self.jc.get_control()
                _, gripper_left, _ = self.jc.get_control()

                # Determine left and right hand states
                left = close_left_hand if gripper_left == 1 else open_left_hand
                right = close_right_hand if gripper_right == 1 else open_right_hand

                # Build ROS message
                msg = robotHandPosition()
                msg.header.stamp = rospy.Time.now()
                msg.left_hand_position = left
                msg.right_hand_position = right

                # Save last message and publish
                self.last_msg = msg
                self.pub.publish(msg)
            else:
                # If publishing stopped, continue sending last message to hold position
                if self.last_msg:
                    self.pub.publish(self.last_msg)

            self.rate.sleep()

        # Graceful shutdown
        self.jc.disconnect()
        rospy.loginfo("GripStatePublisher shut down cleanly.")


if __name__ == "__main__":
    try:
        GripStatePublisher()
    except rospy.ROSInterruptException:
        pass
