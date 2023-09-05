# Install script for directory: /home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/src/my_first_package

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_package.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package/cmake" TYPE FILE FILES
    "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_packageConfig.cmake"
    "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/my_first_packageConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_first_package" TYPE FILE FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/src/my_first_package/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/$(CATKIN_PACKAGE_BIN_DESTINATION)" TYPE PROGRAM FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/publisher_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/$(CATKIN_PACKAGE_BIN_DESTINATION)" TYPE PROGRAM FILES "/home/lattinone/RobotProgramming/Exercises/10 - ROS & Rviz/Exercises/my_first_ros_project/build/my_first_package/catkin_generated/installspace/subscriber_node.py")
endif()

