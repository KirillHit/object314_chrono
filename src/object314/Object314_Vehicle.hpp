// Object314 vehicle model.

#ifndef OBJECT314_VEHICLE_HPP
#define OBJECT314_VEHICLE_HPP

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Definition of an object314 tracked vehicle with segmented tracks.
/// Both single-pin and double-pin track assemblies can be used with this vehicle model.
class CH_MODELS_API Object314_Vehicle : public ChTrackedVehicle {
  public:
    /// Construct the object314 vehicle within an automatically created Chrono system.
    Object314_Vehicle(bool fixed,
                      TrackShoeType shoe_type,
                      DrivelineTypeTV driveline_type,
                      BrakeType brake_type,
                      ChContactMethod contact_method = ChContactMethod::NSC,
                      CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the object314 vehicle within the specified Chrono system.
    Object314_Vehicle(bool fixed,
                      TrackShoeType shoe_type,
                      DrivelineTypeTV driveline_type,
                      BrakeType brake_type,
                      ChSystem* system,
                      CollisionType chassis_collision_type = CollisionType::NONE);

    ~Object314_Vehicle() {}

    /// Create the track shoes (default: true).
    void CreateTrack(bool val) { m_create_track = val; }

    /// Initialize the object314 vehicle at the specified location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed,
                TrackShoeType shoe_type,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                CollisionType chassis_collision_type);

    bool m_create_track;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
