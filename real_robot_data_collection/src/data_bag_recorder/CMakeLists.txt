cmake_minimum_required(VERSION 3.0.2)
project(data_bag_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamic_biped
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/data_bag_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
