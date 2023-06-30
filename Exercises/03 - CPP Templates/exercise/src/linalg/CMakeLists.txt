add_library(vec_f_library
  vec_f.cpp)

add_library(mat_f_library
  mat_f.cpp)

target_link_libraries(mat_f_library
  vec_f_library)

add_executable(vec_f_main
vec_f_main.cpp)

target_link_libraries(vec_f_main
  vec_f_library)

add_executable(mat_f_main
  mat_f_main.cpp)

target_link_libraries(mat_f_main
  mat_f_library
  vec_f_library
  )
