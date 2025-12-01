// Object314 simple engine model based on hyperbolical speed-torque curve (CVT)

#include "Object314_EngineSimple.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// Static variables
const double Object314_EngineSimple::m_max_torque = 2400 / 50;
const double Object314_EngineSimple::m_max_power = 530000 / 50;
const double Object314_EngineSimple::m_max_speed = 10000 / 50;

Object314_EngineSimple::Object314_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
