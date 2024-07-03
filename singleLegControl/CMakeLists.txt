cmake_minimum_required(VERSION 3.10)
project(ArduinoSerialConnection)

set(CMAKE_CXX_STANDARD 11)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed

# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories("/home/hunter/raisim_ws/repos/matplotlib-cpp")


# Add executables
add_executable(singleLegControl singleLegControl.cpp ../modules/custom/arduinoConnection/arduinoController.cpp hexapodLeg.cpp ../modules/custom/rsTimedLoop/rsTimedLoop.cpp)
# add_executable(readArduino readArduino.cpp ../modules/custom/arduinoConnection/arduinoController.cpp)

# Link libraries to executables
target_link_libraries(singleLegControl fmt::fmt ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
# target_link_libraries(readArduino ${Boost_LIBRARIES})
