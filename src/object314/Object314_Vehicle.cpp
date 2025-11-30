// Object314 vehicle model.

#include "Object314_Vehicle.hpp"

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_Chassis.hpp"
#include "Object314_SimpleDriveline.hpp"
#include "track_assembly/Object314_TrackAssemblySinglePin.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Object314_Vehicle::Object314_Vehicle(bool fixed,
                                     TrackShoeType shoe_type,
                                     DrivelineTypeTV driveline_type,
                                     BrakeType brake_type,
                                     ChContactMethod contact_method,
                                     CollisionType chassis_collision_type)
    : ChTrackedVehicle("Object314", contact_method), m_create_track(true) {
    Create(fixed, shoe_type, driveline_type, brake_type, chassis_collision_type);
}

Object314_Vehicle::Object314_Vehicle(bool fixed,
                                     TrackShoeType shoe_type,
                                     DrivelineTypeTV driveline_type,
                                     BrakeType brake_type,
                                     ChSystem* system,
                                     CollisionType chassis_collision_type)
    : ChTrackedVehicle("Object314", system), m_create_track(true) {
    Create(fixed, shoe_type, driveline_type, brake_type, chassis_collision_type);
}

void Object314_Vehicle::Create(bool fixed,
                               TrackShoeType shoe_type,
                               DrivelineTypeTV driveline_type,
                               BrakeType brake_type,
                               CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Object314_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the track assembly subsystems
    switch (shoe_type) {
        case TrackShoeType::SINGLE_PIN:
            m_tracks[0] = chrono_types::make_shared<Object314_TrackAssemblySinglePin>(LEFT, brake_type);
            m_tracks[1] = chrono_types::make_shared<Object314_TrackAssemblySinglePin>(RIGHT, brake_type);
            break;
        case TrackShoeType::DOUBLE_PIN:
            ////m_tracks[0] = chrono_types::make_shared<Object314_TrackAssemblyDoublePin>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Object314_TrackAssemblyDoublePin>(RIGHT, brake_type);
            std::cout << "Unimplemented track assembly model.\n";
            break;
        case TrackShoeType::BAND_BUSHING:
            ////m_tracks[0] = chrono_types::make_shared<Object314_TrackAssemblyBandBushing>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Object314_TrackAssemblyBandBushing>(RIGHT, brake_type);
            std::cout << "Unimplemented track assembly model.\n";
            break;
        case TrackShoeType::BAND_ANCF:
            ////m_tracks[0] = chrono_types::make_shared<Object314_TrackAssemblyBandANCF>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Object314_TrackAssemblyBandANCF>(RIGHT, brake_type);
            std::cout << "Unimplemented track assembly model.\n";
            break;
    }

    // Create the driveline
    switch (driveline_type) {
        case DrivelineTypeTV::SIMPLE:
            m_driveline = chrono_types::make_shared<Object314_SimpleDriveline>();
            break;
        case DrivelineTypeTV::BDS:
            ////m_driveline = chrono_types::make_shared<Object314_DrivelineBDS>();
            std::cout << "Unimplemented driveline model.\n";
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Object314_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(this, chassisPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);

    // Initialize the left and right track assemblies.
    double track_offset = 0.84 / 2.0;
    m_tracks[0]->Initialize(m_chassis, ChVector3d(0, track_offset, 0), m_create_track);
    m_tracks[1]->Initialize(m_chassis, ChVector3d(0, -track_offset, 0), m_create_track);

    // Initialize the driveline subsystem
    m_driveline->Initialize(m_chassis, m_tracks[0], m_tracks[1]);

    // Invoke base class method
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
