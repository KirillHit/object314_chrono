cmake_minimum_required(VERSION 3.16)
project(object314_robot CXX)

set(CMAKE_CXX_STANDARD 17)

set(CH_VEHICLE_DATA_DIR "${CMAKE_SOURCE_DIR}/object314/data" CACHE PATH "Path to vehicle data")

find_package(Chrono 
  COMPONENTS 
    Irrlicht
    VSG
    Multicore
    Sensor
    Vehicle
)

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

set(SRC_FILES
  main.cpp
  vehicle/object314_vehicle.cpp
  powertrain/Object314_EngineSimpleMap.cpp
  powertrain/Object314_EngineSimple.cpp
  powertrain/Object314_EngineShafts.cpp
  powertrain/Object314_AutomaticTransmissionSimpleMap.cpp
  powertrain/Object314_AutomaticTransmissionSimple.cpp
  powertrain/Object314_AutomaticTransmissionShafts.cpp
  chassis/Object314_Chassis.cpp
)

add_executable(object314_robot ${SRC_FILES})

target_link_libraries(object314_robot PRIVATE ${CHRONO_TARGETS})
