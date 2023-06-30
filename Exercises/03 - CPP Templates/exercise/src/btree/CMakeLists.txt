add_library(btree_library btree.cpp)

add_executable(btree_main 
  btree_main.cpp)

target_link_libraries(btree_main
  btree_library)
