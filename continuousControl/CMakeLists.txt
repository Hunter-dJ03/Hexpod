cmake_minimum_required(VERSION 3.10)
project(HexapodControl)

set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed
find_package(raisim CONFIG REQUIRED)
find_package(SDL2 REQUIRED)

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    "/home/hunter/Documents/github/importedRepos/matplotlib-cpp"
    # "/home/hunter/raisim_ws/repos/matplotlib-cpp"
)

# Add executables
add_executable(continuousControl
    continuousControl.cpp
    ../modules/custom/arduinoConnection/arduinoController.cpp
    hexapod.cpp
    ../modules/custom/rsTimedLoop/rsTimedLoop.cpp
    ../modules/custom/utilities/utils.cpp
    ../modules/custom/raisimSimulatorFull/raisimSimulator.cpp
)

# Link libraries to executables
target_link_libraries(continuousControl PUBLIC 
    fmt::fmt
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    SDL2::SDL2
    raisim::raisim
)

if(NOT WIN32)
  target_link_libraries(continuousControl PUBLIC pthread)
else()
  target_link_libraries(continuousControl PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(continuousControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_link_libraries(continuousControl PUBLIC ${PYTHON_LIBRARIES})

file(COPY "../modules/models" DESTINATION ${CMAKE_BINARY_DIR})



# cmake .. -DCMAKE_PREFIX_PATH=/home/hunter/raisim_ws/repos/raisimlib/raisim/linux