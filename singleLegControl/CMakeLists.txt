cmake_minimum_required(VERSION 3.10)
project(HexapodControl)

set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/modules/custom/arduinoConnection
    ${CMAKE_SOURCE_DIR}/modules/custom/rsTimedLoop
    ${CMAKE_SOURCE_DIR}/modules/custom/utilities
    "/home/hunter/raisim_ws/repos/matplotlib-cpp"
)

# Add executables
add_executable(singleLegControl
    singleLegControl.cpp
    ../modules/custom/arduinoConnection/arduinoController.cpp
    hexapodLeg.cpp
    ../modules/custom/rsTimedLoop/rsTimedLoop.cpp
    ../modules/custom/utilities/utils.cpp
)

# Link libraries to executables
target_link_libraries(singleLegControl
    fmt::fmt
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
)
