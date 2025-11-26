// Wrapper classes for modeling an entire Object314 vehicle assembly
// (including the vehicle itself and the powertrain).

#ifndef OBJECT314_HPP
#define OBJECT314_HPP

#include <array>
#include <string>

#include "Object314_Vehicle.hpp"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

class CH_MODELS_API Object314 {
  public:
    Object314();
    Object314(ChSystem* system);

    ~Object314();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }
    void SetCollisionSystemType(ChCollisionSystem::Type collsys_type) { m_collsysType = collsys_type; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }
    void SetWheelCollisionType(bool roadwheel_as_cylinder, bool idler_as_cylinder, bool roller_as_cylinder) {
        m_wheel_cyl = roadwheel_as_cylinder;
        m_idler_cyl = idler_as_cylinder;
        m_roller_cyl = roller_as_cylinder;
    }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }

    void CreateTrack(bool val) { m_create_track = val; }
    void SetEngineType(EngineModelType val) { m_engineType = val; }
    void SetTransmissionType(TransmissionModelType val) { m_transmissionType = val; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChTrackedVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    std::shared_ptr<ChDrivelineTV> GetDriveline() const { return m_vehicle->GetDriveline(); }

    void Initialize();

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSprocketVisualizationType(VisualizationType vis) { m_vehicle->SetSprocketVisualizationType(vis); }
    void SetIdlerVisualizationType(VisualizationType vis) { m_vehicle->SetIdlerVisualizationType(vis); }
    void SetRollerVisualizationType(VisualizationType vis) { m_vehicle->SetRollerVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetIdlerWheelVisualizationType(VisualizationType vis) { m_vehicle->SetIdlerWheelVisualizationType(vis); }
    void SetRoadWheelVisualizationType(VisualizationType vis) { m_vehicle->SetRoadWheelVisualizationType(vis); }
    void SetTrackShoeVisualizationType(VisualizationType vis) { m_vehicle->SetTrackShoeVisualizationType(vis); }

    void Synchronize(double time, const DriverInputs& driver_inputs);
    void Synchronize(double time,
                     const DriverInputs& driver_inputs,
                     const TerrainForces& shoe_forces_left,
                     const TerrainForces& shoe_forces_right);
    void Advance(double step);

    void LogConstraintViolations() { m_vehicle->LogConstraintViolations(); }

  protected:
    ChContactMethod m_contactMethod;
    ChCollisionSystem::Type m_collsysType;
    CollisionType m_chassisCollisionType;
    bool m_fixed;
    bool m_create_track;
    bool m_wheel_cyl;
    bool m_idler_cyl;
    bool m_roller_cyl;

    BrakeType m_brake_type;
    TrackShoeType m_shoe_type;
    DrivelineTypeTV m_driveline_type;
    EngineModelType m_engineType;
    TransmissionModelType m_transmissionType;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    Object314_Vehicle* m_vehicle;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
