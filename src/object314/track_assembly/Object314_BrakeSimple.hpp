// Object314 simple brake model

#ifndef OBJECT314_BRAKE_SIMPLE_HPP
#define OBJECT314_BRAKE_SIMPLE_HPP

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Empty brake stub required by Chrono track assembly API. Object314 is controlled by sprocket torques.
class CH_MODELS_API Object314_BrakeSimple : public ChTrackBrakeSimple {
  public:
    Object314_BrakeSimple(const std::string& name) : ChTrackBrakeSimple(name) {}
    ~Object314_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return 0.0; }
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
