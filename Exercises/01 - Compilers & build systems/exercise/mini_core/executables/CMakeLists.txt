# Compile executable that will be run  
add_executable(many_object_in_stack_example
	many_object_in_stack_example.cpp
)

# We tell CMake that we need to link a libary called "core_library"
# to our main executable "many_object_in_stack_example"
target_link_libraries(many_object_in_stack_example
	core_library
)
