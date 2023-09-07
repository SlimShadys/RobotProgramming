# Install script for directory: /home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/src/my_first_package

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/msg" TYPE FILE FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/src/my_first_package/msg/position.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/srv" TYPE FILE FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/src/my_first_package/srv/multiplier.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/cmake" TYPE FILE FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_package-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/include/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/share/roseus/ros/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/share/common-lisp/ros/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/share/gennodejs/ros/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/lib/python3/dist-packages/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/devel/lib/python3/dist-packages/my_first_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_package.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/cmake" TYPE FILE FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_package-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/cmake" TYPE FILE FILES
    "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_packageConfig.cmake"
    "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_packageConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package" TYPE FILE FILES "/home/lattinone/Desktop/RobotProgrammingPersonal/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/src/my_first_package/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/$(CATKIN_PACKAGE_BIN_DESTINATION)" TYPE PROGRAM FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/publisher_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/$(CATKIN_PACKAGE_BIN_DESTINATION)" TYPE PROGRAM FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/subscriber_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/$(CATKIN_PACKAGE_BIN_DESTINATION)" TYPE PROGRAM FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS and Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/service_server_node.py")
endif()

