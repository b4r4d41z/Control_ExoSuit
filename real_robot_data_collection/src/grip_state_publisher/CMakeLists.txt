cmake_minimum_required(VERSION 3.0.2)
project(grip_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  dynamic_biped
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/grip_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
