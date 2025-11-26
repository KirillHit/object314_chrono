// Wrapper classes for modeling an entire Object314 vehicle assembly
// (including the vehicle itself and the powertrain).

#include "Object314.hpp"
#include "chrono/ChConfig.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "powertrain/Object314_AutomaticTransmissionShafts.hpp"
#include "powertrain/Object314_AutomaticTransmissionSimple.hpp"
#include "powertrain/Object314_AutomaticTransmissionSimpleMap.hpp"
#include "powertrain/Object314_EngineShafts.hpp"
#include "powertrain/Object314_EngineSimple.hpp"
#include "powertrain/Object314_EngineSimpleMap.hpp"

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
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
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
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
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
        m_vehicle = new Object314_Vehicle(m_fixed, m_shoe_type, m_driveline_type, m_brake_type, m_system,
                                          m_chassisCollisionType);
    } else {
        m_vehicle = new Object314_Vehicle(m_fixed, m_shoe_type, m_driveline_type, m_brake_type, m_contactMethod,
                                          m_chassisCollisionType);
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

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;
    switch (m_engineType) {
        case EngineModelType::SHAFTS:
            engine = chrono_types::make_shared<Object314_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<Object314_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<Object314_EngineSimple>("Engine");
            transmission = chrono_types::make_shared<Object314_AutomaticTransmissionSimple>(
                "Transmiss"
                "ion");
            break;
    }

    if (!transmission) {
        switch (m_transmissionType) {
            case TransmissionModelType::AUTOMATIC_SHAFTS:
                transmission = chrono_types::make_shared<Object314_AutomaticTransmissionShafts>(
                    "Trans"
                    "missi"
                    "on");
                break;
            case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
                transmission = chrono_types::make_shared<Object314_AutomaticTransmissionSimpleMap>("Transmission");
                break;
            default:
                break;
        }
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------

void Object314::Synchronize(double time, const DriverInputs& driver_inputs) {
    m_vehicle->Synchronize(time, driver_inputs);
}

void Object314::Synchronize(double time,
                            const DriverInputs& driver_inputs,
                            const TerrainForces& shoe_forces_left,
                            const TerrainForces& shoe_forces_right) {
    m_vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
}

void Object314::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace object314
}  // namespace vehicle
}  // namespace chrono
