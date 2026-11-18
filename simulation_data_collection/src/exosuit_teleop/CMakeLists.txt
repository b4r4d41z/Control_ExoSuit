cmake_minimum_required(VERSION 3.0.2)
project(exosuit_teleop)
find_package(catkin REQUIRED COMPONENTS dynamic_biped rosgraph rospy sensor_msgs std_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS dynamic_biped rosgraph rospy sensor_msgs std_msgs)
catkin_install_python(PROGRAMS scripts/exosuit_teleop_node.py scripts/teleop_log_view.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_test(NAME exosuit_teleop_unit
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_*.py)
endif()
