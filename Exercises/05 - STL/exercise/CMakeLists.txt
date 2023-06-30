cmake_minimum_required(VERSION 2.8.3)
project(lesson_3)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# add project source code
add_subdirectory(src/02_mat_and_vec)

# TODO do the same for executables
include_directories(src src/02_mat_and_vec)

add_executable(point_multisort_gen_data 
    src/point_multisort_gen_data.cpp)
add_executable(point_multisort 
    src/point_multisort.cpp)
add_executable(int_sort 
    src/int_sort.cpp)
add_executable(splitter_test 
    src/splitter_test.cpp)
add_executable(binary_tree_template_test 
    src/binary_tree_template_test.cpp)