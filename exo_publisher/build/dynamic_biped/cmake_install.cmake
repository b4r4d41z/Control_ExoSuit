# Install script for directory: /home/ExoSuit/exo_publisher/src/dynamic_biped

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ExoSuit/exo_publisher/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE PROGRAM FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE PROGRAM FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/setup.bash;/home/ExoSuit/exo_publisher/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/setup.bash"
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/setup.sh;/home/ExoSuit/exo_publisher/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/setup.sh"
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/setup.zsh;/home/ExoSuit/exo_publisher/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/setup.zsh"
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/setup.fish;/home/ExoSuit/exo_publisher/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/setup.fish"
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ExoSuit/exo_publisher/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ExoSuit/exo_publisher/install" TYPE FILE FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/msg" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/armHandPose.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/ECJointMotordata.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/handRotationEular.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/handRotation.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/QuaternionArray.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/recordArmHandPose.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotArmInfo.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotArmQVVD.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robot_hand_eff.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotHandPosition.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotHeadMotionData.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotPhase.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotQVTau.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/robotTorsoState.msg"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/msg/walkCommand.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/srv" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/changeAMBACCtrlMode.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/changeArmCtrlMode.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/controlEndHand.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/srvchangeCtlMode.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/srvChangeJoller.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/srvChangePhases.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/srvClearPositionCMD.srv"
    "/home/ExoSuit/exo_publisher/src/dynamic_biped/srv/srvManiInst.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/dynamic_biped-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/include/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/share/roseus/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/share/common-lisp/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/share/gennodejs/ros/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/lib/python3/dist-packages/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ExoSuit/exo_publisher/devel/.private/dynamic_biped/lib/python3/dist-packages/dynamic_biped")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/dynamic_biped.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/dynamic_biped-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped/cmake" TYPE FILE FILES
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/dynamic_bipedConfig.cmake"
    "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/dynamic_bipedConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamic_biped" TYPE FILE FILES "/home/ExoSuit/exo_publisher/src/dynamic_biped/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dynamic_biped" TYPE PROGRAM FILES "/home/ExoSuit/exo_publisher/build/dynamic_biped/catkin_generated/installspace/arm_move_X.py")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ExoSuit/exo_publisher/build/dynamic_biped/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ExoSuit/exo_publisher/build/dynamic_biped/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
