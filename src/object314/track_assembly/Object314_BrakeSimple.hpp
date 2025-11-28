// Object314 simple brake model

#ifndef OBJECT314_BRAKE_SIMPLE_HPP
#define OBJECT314_BRAKE_SIMPLE_HPP

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Simple Object314 brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Object314_BrakeSimple : public ChTrackBrakeSimple {
  public:
    Object314_BrakeSimple(const std::string& name) : ChTrackBrakeSimple(name) {}
    ~Object314_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return 30000.0; }
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
