cmake_minimum_required(VERSION 3.0.2)
project(dynamic_biped)

## Use catkin and find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Declare message files
add_message_files(
  FILES
  armHandPose.msg
  ECJointMotordata.msg
  handRotationEular.msg
  handRotation.msg
  QuaternionArray.msg
  recordArmHandPose.msg
  robotArmInfo.msg
  robotArmQVVD.msg
  robot_hand_eff.msg
  robotHandPosition.msg
  robotHeadMotionData.msg
  robotPhase.msg
  robotQVTau.msg
  robotTorsoState.msg
  walkCommand.msg
)

## Declare service files
add_service_files(
  FILES
  changeAMBACCtrlMode.srv
  changeArmCtrlMode.srv
  controlEndHand.srv
  srvchangeCtlMode.srv
  srvChangeJoller.srv
  srvChangePhases.srv
  srvClearPositionCMD.srv
  srvManiInst.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

#############
## Install ##
#############

## Install Python scripts
catkin_install_python(PROGRAMS
  script/arm_move_X.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
