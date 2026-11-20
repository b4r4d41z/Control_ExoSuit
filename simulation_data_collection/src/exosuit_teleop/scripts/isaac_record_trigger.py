#!/usr/bin/env python3
"""Toggle Isaac recording commands from the right Joy-Con's X button."""

from threading import RLock

import rospy
from std_msgs.msg import Bool

from exosuit_teleop.joycon_input import JoyCon


def main():
    rospy.init_node("isaac_record_trigger")
    publisher = rospy.Publisher(
        "/data_collection/recording", Bool, queue_size=1, latch=True
    )
    publisher.publish(Bool(data=False))
    rospy.loginfo("Isaac recording command: OFF")

    jc = None
    cleaned_up = False
    command_lock = RLock()

    def cleanup():
        nonlocal cleaned_up
        with command_lock:
            if cleaned_up:
                return
            cleaned_up = True
            try:
                publisher.publish(Bool(data=False))
                rospy.loginfo("Isaac recording command: OFF")
            except rospy.ROSException as error:
                rospy.logwarn("Could not publish final recording command: %s", error)

    try:
        jc = JoyCon("right")
        rospy.on_shutdown(cleanup)
        rospy.loginfo("Press 'X' on the right Joy-Con to toggle Isaac recording.")

        recording = False
        button_last = 0
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            jc.check()
            state = jc.button.get_button_x()
            if state == 1 and button_last == 0:
                with command_lock:
                    if cleaned_up:
                        break
                    recording = not recording
                    publisher.publish(Bool(data=recording))
                    rospy.loginfo("Isaac recording command: %s", "ON" if recording else "OFF")
            button_last = state
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cleanup()
        if jc is not None:
            jc.close()


if __name__ == "__main__":
    main()
