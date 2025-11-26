// Automatic transmssion model for the Object314 vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)

#include "Object314_AutomaticTransmissionSimpleMap.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

const double rpm2rads = CH_PI / 30;

Object314_AutomaticTransmissionSimpleMap::Object314_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Object314_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(0.1708);
    fwd.push_back(0.2791);
    fwd.push_back(0.4218);
    fwd.push_back(0.6223);
    fwd.push_back(1.0173);
    fwd.push_back(1.5361);
}

void Object314_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2400 * rpm2rads));
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
