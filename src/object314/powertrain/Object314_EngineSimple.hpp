// Object314 simple engine model based on hyperbolical speed-torque curve (CVT)

#ifndef OBJECT314_ENGINE_SIMPLE_HPP
#define OBJECT314_ENGINE_SIMPLE_HPP

#include "chrono_vehicle/powertrain/EngineSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 simple engine model based on hyperbolical speed-torque curve (CVT).
class CH_MODELS_API Object314_EngineSimple : public ChEngineSimple {
  public:
    Object314_EngineSimple(const std::string& name);

    ~Object314_EngineSimple() {}

    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;  ///< maximum motor torque
    static const double m_max_power;   ///< maximum motor power
    static const double m_max_speed;   ///< maximum engine speed
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
