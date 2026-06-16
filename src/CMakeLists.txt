cmake_minimum_required(VERSION 3.16)
project(object314_robot CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Chrono
  COMPONENTS
    Irrlicht
    Multicore
    Sensor
    Vehicle
)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_nmpc_interfaces REQUIRED)

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

add_executable(object314_ros_sim ros_sim.cpp)

target_link_libraries(object314_ros_sim
  PRIVATE
    object314_vehicle
    rclcpp::rclcpp
    vehicle_nmpc_interfaces::vehicle_nmpc_interfaces__rosidl_typesupport_cpp
)

install(
  TARGETS
    object314_teleop
    object314_ros_sim
  RUNTIME DESTINATION
    share/chrono/bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/object314/data/
        DESTINATION share/chrono/data/vehicle/object314)
