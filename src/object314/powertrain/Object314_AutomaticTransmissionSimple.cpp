// Automatic transmssion model for the Object314 vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)

#include "Object314_AutomaticTransmissionSimple.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

const double rpm2rads = CH_PI / 30;

Object314_AutomaticTransmissionSimple::Object314_AutomaticTransmissionSimple(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Object314_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(0.1708);
}

void Object314_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
