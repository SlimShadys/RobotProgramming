cmake_minimum_required(VERSION 2.8.3)
project(lesson_6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)


include_directories(. /usr/include/eigen3)

add_executable(eigen_icp_2d_test eigen_icp_2d_test.cpp eigen_icp_2d.cpp )
