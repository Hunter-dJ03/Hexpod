cmake_minimum_required(VERSION 3.10)

# Set the project name
project(ReadControllerInput)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(read_controller read_controller.cpp)

# Add necessary libraries
target_link_libraries(read_controller PRIVATE pthread)
