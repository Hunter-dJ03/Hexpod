cmake_minimum_required(VERSION 3.10)
project(ArduinoSerialConnection)

set(CMAKE_CXX_STANDARD 11)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Add executables
add_executable(arduinoConnection arduinoConnection.cpp arduinoController.cpp hexapodLeg.cpp rs_timed_loop.cpp)
add_executable(readArduino readArduino.cpp arduinoController.cpp)

# Link libraries to executables
target_link_libraries(arduinoConnection fmt::fmt ${Boost_LIBRARIES})
target_link_libraries(readArduino ${Boost_LIBRARIES})
