#!/usr/bin/env python3
import rospy
import time
import subprocess
import os
from joyconrobotics import JoyconRobotics

def main():
    rospy.init_node("data_bag_recorder", anonymous=True)
    jc = JoyconRobotics("right", without_rest_init=True)
    
    # Save .bag files to external disk, keep per-project folder
    project_name = "simulation_data_collection"
    save_dir = os.path.join("/mnt/data_disk", project_name, "records")
    os.makedirs(save_dir, exist_ok=True)


    recording = False
    process = None
    button_last = 0

    topics = rospy.get_param("~topics", [
        "/head_camera/color/image_raw/compressed",
        "/head_camera/color/camera_info",
        "/left_hand_camera/image_raw/compressed",
        "/left_hand_camera/camera_info",
        "/right_hand_camera/image_raw/compressed",
        "/right_hand_camera/camera_info",
        "/control_robot_hand_position",
        "/kuavo_arm_traj",
        "/robot_arm_q_v_tau",
        "/robot_hand_position"
    ])
    
    button_name = "x"

    rospy.loginfo("Press 'X' to start/stop recording data.")

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
