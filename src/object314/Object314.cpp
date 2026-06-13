// Wrapper classes for modeling an entire Object314 vehicle assembly.

#include "Object314.hpp"
#include "chrono/ChConfig.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
Object314::Object314()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_roller_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

Object314::Object314(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_roller_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

Object314::~Object314() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void Object314::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Object314::Initialize() {
    // Create and initialize the Object314 vehicle
    if (m_system) {
        m_vehicle = new Object314_Vehicle(m_fixed, m_shoe_type, m_system, m_chassisCollisionType);
    } else {
        m_vehicle = new Object314_Vehicle(m_fixed, m_shoe_type, m_contactMethod, m_chassisCollisionType);
    }
    m_vehicle->SetCollisionSystemType(m_collsysType);
    m_vehicle->CreateTrack(m_create_track);
    m_vehicle->GetTrackAssembly(LEFT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, m_roller_cyl);
    m_vehicle->GetTrackAssembly(RIGHT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, m_roller_cyl);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------

void Object314::Synchronize(double time, const DriverInputs& driver_inputs) {
    m_vehicle->Synchronize(time, driver_inputs);
}

void Object314::ApplyTrackTorques(double left_torque, double right_torque) {
    m_vehicle->GetTrackAssembly(LEFT)->GetSprocket()->ApplyAxleTorque(left_torque);
    m_vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->ApplyAxleTorque(right_torque);
}

void Object314::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace object314
}  // namespace vehicle
}  // namespace chrono
