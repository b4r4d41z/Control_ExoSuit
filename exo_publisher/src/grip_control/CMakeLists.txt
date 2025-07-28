cmake_minimum_required(VERSION 3.0.2)
project(grip_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  dynamic_biped
  message_generation
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs dynamic_biped
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
