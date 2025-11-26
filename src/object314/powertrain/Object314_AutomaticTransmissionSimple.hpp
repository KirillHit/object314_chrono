// Automatic transmssion model for the Object314 vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)

#ifndef OBJECT314_AUTOMATIC_TRANSMISSION_SIMPLE_HPP
#define OBJECT314_AUTOMATIC_TRANSMISSION_SIMPLE_HPP

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 automatic transmission model template based on a simple gear-shifting model.
class CH_MODELS_API Object314_AutomaticTransmissionSimple : public ChAutomaticTransmissionSimpleMap {
  public:
    Object314_AutomaticTransmissionSimple(const std::string& name);
    ~Object314_AutomaticTransmissionSimple() {}

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify the min and max engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
