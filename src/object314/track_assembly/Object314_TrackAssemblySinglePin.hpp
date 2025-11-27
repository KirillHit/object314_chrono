// Object314 single-pin track assembly subsystem.

#ifndef OBJECT314_TRACK_ASSEMBLY_SINGLE_PIN_HPP
#define OBJECT314_TRACK_ASSEMBLY_SINGLE_PIN_HPP

#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 track assembly using single-pin track shoes.
class CH_MODELS_API Object314_TrackAssemblySinglePin : public ChTrackAssemblySinglePin {
  public:
    Object314_TrackAssemblySinglePin(VehicleSide side, BrakeType brake_type);

    virtual const ChVector3d GetSprocketLocation() const override;
    virtual const ChVector3d GetIdlerLocation() const override;
    virtual const ChVector3d GetRoadWhelAssemblyLocation(int which) const override;
    virtual const ChVector3d GetRollerLocation(int which) const override;

  private:
    static const ChVector3d m_sprocket_loc;
    static const ChVector3d m_idler_loc;
    static const ChVector3d m_susp_locs_L[6];
    static const ChVector3d m_susp_locs_R[6];
    static const ChVector3d m_supp_locs_L[3];
    static const ChVector3d m_supp_locs_R[3];

    static const double m_right_x_offset;
};

/// @} vehicle_models_Object314

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
