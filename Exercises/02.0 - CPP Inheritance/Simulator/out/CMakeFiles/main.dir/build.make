# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out"

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/simple_geometry.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/simple_geometry.cpp.o: ../src/simple_geometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/simple_geometry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/simple_geometry.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/simple_geometry.cpp"

CMakeFiles/main.dir/src/simple_geometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/simple_geometry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/simple_geometry.cpp" > CMakeFiles/main.dir/src/simple_geometry.cpp.i

CMakeFiles/main.dir/src/simple_geometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/simple_geometry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/simple_geometry.cpp" -o CMakeFiles/main.dir/src/simple_geometry.cpp.s

CMakeFiles/main.dir/src/world.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/world.cpp.o: ../src/world.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/world.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/world.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/world.cpp"

CMakeFiles/main.dir/src/world.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/world.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/world.cpp" > CMakeFiles/main.dir/src/world.cpp.i

CMakeFiles/main.dir/src/world.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/world.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/world.cpp" -o CMakeFiles/main.dir/src/world.cpp.s

CMakeFiles/main.dir/src/robot.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/robot.cpp.o: ../src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/robot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/robot.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/robot.cpp"

CMakeFiles/main.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/robot.cpp" > CMakeFiles/main.dir/src/robot.cpp.i

CMakeFiles/main.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/robot.cpp" -o CMakeFiles/main.dir/src/robot.cpp.s

CMakeFiles/main.dir/src/lidar.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/lidar.cpp.o: ../src/lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/src/lidar.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/lidar.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/lidar.cpp"

CMakeFiles/main.dir/src/lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/lidar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/lidar.cpp" > CMakeFiles/main.dir/src/lidar.cpp.i

CMakeFiles/main.dir/src/lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/lidar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/lidar.cpp" -o CMakeFiles/main.dir/src/lidar.cpp.s

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/main.cpp"

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/main.cpp" > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/main.cpp" -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/pan_unit.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/pan_unit.cpp.o: ../src/pan_unit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/main.dir/src/pan_unit.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/pan_unit.cpp.o -c "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/pan_unit.cpp"

CMakeFiles/main.dir/src/pan_unit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/pan_unit.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/pan_unit.cpp" > CMakeFiles/main.dir/src/pan_unit.cpp.i

CMakeFiles/main.dir/src/pan_unit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/pan_unit.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/pan_unit.cpp" -o CMakeFiles/main.dir/src/pan_unit.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/simple_geometry.cpp.o" \
"CMakeFiles/main.dir/src/world.cpp.o" \
"CMakeFiles/main.dir/src/robot.cpp.o" \
"CMakeFiles/main.dir/src/lidar.cpp.o" \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/pan_unit.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/simple_geometry.cpp.o
main: CMakeFiles/main.dir/src/world.cpp.o
main: CMakeFiles/main.dir/src/robot.cpp.o
main: CMakeFiles/main.dir/src/lidar.cpp.o
main: CMakeFiles/main.dir/src/main.cpp.o
main: CMakeFiles/main.dir/src/pan_unit.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/local/lib/libopencv_gapi.so.4.8.0
main: /usr/local/lib/libopencv_highgui.so.4.8.0
main: /usr/local/lib/libopencv_ml.so.4.8.0
main: /usr/local/lib/libopencv_objdetect.so.4.8.0
main: /usr/local/lib/libopencv_photo.so.4.8.0
main: /usr/local/lib/libopencv_stitching.so.4.8.0
main: /usr/local/lib/libopencv_video.so.4.8.0
main: /usr/local/lib/libopencv_videoio.so.4.8.0
main: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
main: /usr/local/lib/libopencv_dnn.so.4.8.0
main: /usr/local/lib/libopencv_calib3d.so.4.8.0
main: /usr/local/lib/libopencv_features2d.so.4.8.0
main: /usr/local/lib/libopencv_flann.so.4.8.0
main: /usr/local/lib/libopencv_imgproc.so.4.8.0
main: /usr/local/lib/libopencv_core.so.4.8.0
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator" "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator" "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out" "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out" "/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/out/CMakeFiles/main.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend
