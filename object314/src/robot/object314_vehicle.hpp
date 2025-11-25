#ifndef OBJECT314_VEHICLE_HPP
#define OBJECT314_VEHICLE_HPP

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {


class CH_MODELS_API Marder_Vehicle : public ChTrackedVehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    Marder_Vehicle(bool fixed,
                   TrackShoeType shoe_type,
                   DrivelineTypeTV driveline_type,
                   BrakeType brake_type,
                   ChContactMethod contact_method = ChContactMethod::NSC,
                   CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    Marder_Vehicle(bool fixed,
                   TrackShoeType shoe_type,
                   DrivelineTypeTV driveline_type,
                   BrakeType brake_type,
                   ChSystem* system,
                   CollisionType chassis_collision_type = CollisionType::NONE);

    ~Marder_Vehicle() {}

    /// Create the track shoes (default: true).
    void CreateTrack(bool val) { m_create_track = val; }

    /// Initialize the M113 vehicle at the specified location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed,
                TrackShoeType shoe_type,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                CollisionType chassis_collision_type);

    bool m_create_track;
};

/// @} vehicle_models_marder

}  // namespace marder

#endif