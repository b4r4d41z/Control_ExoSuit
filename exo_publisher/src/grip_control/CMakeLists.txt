cmake_minimum_required(VERSION 3.0.2)
project(grip_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  dynamic_biped   # твой пакет с robot_hand_eff.msg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/joycon_hand_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
