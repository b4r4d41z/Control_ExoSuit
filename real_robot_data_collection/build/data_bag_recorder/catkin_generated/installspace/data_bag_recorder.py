#!/usr/bin/env python3
import rospy
import time
import subprocess
import os
from joyconrobotics import JoyconRobotics

def main():
    rospy.init_node("data_bag_recorder", anonymous=True)
    jc = JoyconRobotics("right", without_rest_init=True)
    
    # UPDATED PATH: save .bag files to the 'records' folder in project root
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    project_root = os.path.abspath(os.path.join(package_dir, "../../.."))
    save_dir = os.path.join(project_root, "records")
    os.makedirs(save_dir, exist_ok=True)

    recording = False
    process = None
    button_last = 0

    topics = rospy.get_param("~topics", [
        "/control_robot_hand_position",
        "/kuavo_arm_traj",
        "/grip_state"
    ])
    button_name = "x"

    rospy.loginfo("Press button '%s' to toggle recording", button_name)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        state = jc.button.get_button_x()

        if state == 1 and button_last == 0:
            if not recording:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                bag_path = os.path.join(save_dir, f"record_{timestamp}.bag")
                cmd = ["rosbag", "record", "-O", bag_path] + topics
                process = subprocess.Popen(cmd)
                recording = True
                rospy.loginfo("Started recording → %s", bag_path)
            else:
                rospy.loginfo("Stopping recording …")
                if process:
                    process.terminate()
                    process.wait()
                recording = False
                process = None
                rospy.loginfo("Recording stopped and saved.")

        button_last = state
        rate.sleep()

    if recording and process:
        process.terminate()
        process.wait()
    jc.disconnect()

if __name__ == "__main__":
    main()
