// Object314 vehicle model.

#include "Object314_Vehicle.hpp"

#include <stdexcept>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_Chassis.hpp"
#include "track_assembly/Object314_TrackAssemblySinglePin.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Object314_Vehicle::Object314_Vehicle(bool fixed,
                                     TrackShoeType shoe_type,
                                     ChContactMethod contact_method,
                                     CollisionType chassis_collision_type)
    : ChTrackedVehicle("Object314", contact_method), m_create_track(true) {
    Create(fixed, shoe_type, chassis_collision_type);
}

Object314_Vehicle::Object314_Vehicle(bool fixed,
                                     TrackShoeType shoe_type,
                                     ChSystem* system,
                                     CollisionType chassis_collision_type)
    : ChTrackedVehicle("Object314", system), m_create_track(true) {
    Create(fixed, shoe_type, chassis_collision_type);
}

void Object314_Vehicle::Create(bool fixed, TrackShoeType shoe_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Object314_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the track assembly subsystems
    switch (shoe_type) {
        case TrackShoeType::SINGLE_PIN:
            m_tracks[0] = chrono_types::make_shared<Object314_TrackAssemblySinglePin>(LEFT);
            m_tracks[1] = chrono_types::make_shared<Object314_TrackAssemblySinglePin>(RIGHT);
            break;
        case TrackShoeType::DOUBLE_PIN:
            throw std::invalid_argument("Unsupported Object314 track assembly model: DOUBLE_PIN");
        case TrackShoeType::BAND_BUSHING:
            throw std::invalid_argument("Unsupported Object314 track assembly model: BAND_BUSHING");
        case TrackShoeType::BAND_ANCF:
            throw std::invalid_argument("Unsupported Object314 track assembly model: BAND_ANCF");
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

    // Invoke base class method
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
