cmake_minimum_required(VERSION 2.8.3)
project(lesson_6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)


include_directories(. /usr/include/eigen3)

add_executable(eigen_00_basics_test eigen_00_basics_test.cpp)
add_executable(eigen_01_point_loading_test eigen_01_point_loading_test.cpp)
add_executable(transform_points_test transform_points_test.cpp)
