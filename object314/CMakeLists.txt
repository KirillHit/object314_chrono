cmake_minimum_required(VERSION 3.16)
project(my_tracked_robot CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Chrono CONFIG REQUIRED
    ChronoEngine
    ChronoVehicle
    ChronoModels
)

add_executable(my_tracked_robot
    src/main.cpp

    src/chassis/MyChassis.cpp
    src/track/MyTrackAssembly.cpp
    src/track/MyTrackShoe.cpp
    src/wheels/MySprocket.cpp
    src/wheels/MyIdler.cpp
    src/suspension/MyRoadWheel.cpp
    src/suspension/MyRoadWheelAssembly.cpp
    src/utils/Materials.cpp
    src/robot/MyTrackedRobot.cpp
)

target_include_directories(my_tracked_robot PRIVATE src)
target_link_libraries(my_tracked_robot PRIVATE ChronoEngine ChronoVehicle ChronoModels)

file(COPY config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})