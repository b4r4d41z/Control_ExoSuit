cmake_minimum_required(VERSION 3.0.2)
project(servo_angle_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/servo_angle_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
