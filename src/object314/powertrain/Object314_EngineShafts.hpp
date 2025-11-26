// multi role engine model based on ChShaft objects.

#ifndef OBJECT314_ENGINE_SHAFTS_HPP
#define OBJECT314_ENGINE_SHAFTS_HPP

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// @addtogroup vehicle_models_Object314
/// @{

/// Shafts-based engine model for the Object314 vehicle.
class CH_MODELS_API Object314_EngineShafts : public ChEngineShafts {
  public:
    Object314_EngineShafts(const std::string& name);

    ~Object314_EngineShafts() {}

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunctionInterp>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_motorshaft_inertia;
};

/// @} vehicle_models_Object314

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
