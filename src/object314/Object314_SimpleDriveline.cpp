// A simplified Object314 driveline.

#include "Object314_SimpleDriveline.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Object314_SimpleDriveline::m_diff_maxBias = 1;  //// 3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Object314_SimpleDriveline::Object314_SimpleDriveline() : ChSimpleTrackDriveline("Object314_SimpleDriveline") {}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
