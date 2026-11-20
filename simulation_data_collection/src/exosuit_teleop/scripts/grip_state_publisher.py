#!/usr/bin/env python3
from contextlib import ExitStack

import rospy
from dynamic_biped.msg import robotHandPosition

from exosuit_teleop.joycon_input import JoyCon
from exosuit_teleop.ros_adapter import require_simulation_master

open_left_hand = [0, 100, 0, 0, 0, 0]
close_left_hand = [69, 99, 42, 54, 61, 60]

open_right_hand = [0, 100, 0, 0, 0, 0]
close_right_hand = [69, 99, 42, 44, 61, 60]


class GripStatePublisher:
    def __init__(self, right, left):
        self.pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
        self.rate = rospy.Rate(50)

        self.jc_right = right
        self.jc_left = left

        rospy.loginfo("Press 'Y' on the right Joy-Con to start/stop publishing grip state data.")

        self.publishing = False
        self.last_button_state = 0
        self.last_msg = None

        self.left_closed = False
        self.right_closed = False
        self.prev_gripper_left = None
        self.prev_gripper_right = None

    def loop(self):
        while not rospy.is_shutdown():
            self.jc_right.check()
            self.jc_left.check()
            button_state = self.jc_right.button.get_button_y()

            if button_state == 1 and self.last_button_state == 0:
                self.publishing = not self.publishing
                if self.publishing:
                    rospy.loginfo("Publishing started.")
                    _, g_right, _ = self.jc_right.get_control()
                    _, g_left, _ = self.jc_left.get_control()
                    self.prev_gripper_left = g_left
                    self.prev_gripper_right = g_right
                else:
                    rospy.loginfo("Publishing stopped. Holding last state.")
                    self.prev_gripper_left = None
                    self.prev_gripper_right = None

            self.last_button_state = button_state

            if self.publishing:
                _, g_right, _ = self.jc_right.get_control()
                _, g_left, _ = self.jc_left.get_control()

                if self.prev_gripper_left is None:
                    self.prev_gripper_left = g_left
                if self.prev_gripper_right is None:
                    self.prev_gripper_right = g_right

                if self.prev_gripper_left == 0 and g_left == 1:
                    self.left_closed = not self.left_closed
                if self.prev_gripper_right == 0 and g_right == 1:
                    self.right_closed = not self.right_closed

                self.prev_gripper_left = g_left
                self.prev_gripper_right = g_right

                left  = close_left_hand  if self.left_closed  else open_left_hand
                right = close_right_hand if self.right_closed else open_right_hand

                msg = robotHandPosition()
                msg.header.stamp = rospy.Time.now()
                msg.left_hand_position  = left
                msg.right_hand_position = right

                self.last_msg = msg
                self.pub.publish(msg)
            else:
                if self.last_msg:
                    self.pub.publish(self.last_msg)

            self.rate.sleep()

def main():
    rospy.init_node("grip_state_publisher")
    require_simulation_master()
    with ExitStack() as resources:
        right = resources.enter_context(JoyCon("right"))
        left = resources.enter_context(JoyCon("left"))
        GripStatePublisher(right, left).loop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
