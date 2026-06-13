add_library(object314_vehicle STATIC
  Object314.cpp
  Object314_Vehicle.cpp
  Object314_Chassis.cpp
  track_assembly/Object314_Idler.cpp
  track_assembly/Object314_IdlerWheel.cpp
  track_assembly/Object314_RoadWheel.cpp
  track_assembly/Object314_SprocketSinglePin.cpp
  track_assembly/Object314_SupportRoller.cpp
  track_assembly/Object314_Suspension.cpp
  track_assembly/Object314_TrackAssemblySinglePin.cpp
  track_assembly/Object314_TrackShoeSinglePin.cpp
)

target_link_libraries(object314_vehicle
  PUBLIC
    Chrono::Chrono_core
    Chrono::Chrono_vehicle
    Chrono::Chrono_irrlicht
    Chrono::Chrono_vehicle_irrlicht
)

install(TARGETS object314_vehicle
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
