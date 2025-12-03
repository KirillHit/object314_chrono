cmake_minimum_required(VERSION 3.16)
project(object314_robot CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Chrono 
  COMPONENTS
    Irrlicht
    VSG
    Multicore
    Sensor
    Vehicle
)
find_package(Eigen3 REQUIRED)

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

add_subdirectory(object314)

add_executable(object314_teleop main.cpp)

target_link_libraries(object314_teleop
  PRIVATE
    object314_vehicle
)

add_executable(object314_pid pid_control.cpp)

target_link_libraries(object314_pid
  PRIVATE
    object314_vehicle
)

install(
  TARGETS
    object314_teleop
    object314_pid 
  RUNTIME DESTINATION 
    /usr/local/share/chrono/bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/object314/data/
        DESTINATION /usr/local/share/chrono/data/vehicle/object314)