cmake_minimum_required(VERSION 3.16)
project(object314_robot CXX)

set(CMAKE_CXX_STANDARD 17)

set(OBJECT314_DATA_DIR "${CMAKE_INSTALL_PREFIX}/share/object314/data/")
add_compile_definitions(OBJECT314_DATA_DIR="${OBJECT314_DATA_DIR}")

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

install(TARGETS object314_teleop RUNTIME DESTINATION bin)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/object314/data/
        DESTINATION share/object314/data)