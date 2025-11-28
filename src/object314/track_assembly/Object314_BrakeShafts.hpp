// Object314 shafts-based brake model

#ifndef OBJECT314_BRAKE_SHAFTS_HPP
#define OBJECT314_BRAKE_SHAFTS_HPP

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Shafts-based Object314 brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API Object314_BrakeShafts : public ChTrackBrakeShafts {
  public:
    Object314_BrakeShafts(const std::string& name) : ChTrackBrakeShafts(name) {}
    ~Object314_BrakeShafts() {}

    virtual double GetMaxBrakingTorque() override { return 30000.0; }
    virtual double GetShaftInertia() override { return 0.4; }
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
