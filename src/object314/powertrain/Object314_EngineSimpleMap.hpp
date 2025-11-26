// Simple engine model for the Object314 vehicle based on torque-speed engine maps

#ifndef OBJECT314_ENGINE_SIMPLEMAP_HPP
#define OBJECT314_ENGINE_SIMPLEMAP_HPP

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 simple speed-torque engine map subsystem.
class CH_MODELS_API Object314_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Object314_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunctionInterp::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
                                     ChFunctionInterp& mapF   ///< [out] engine map at full throttle
                                     ) override;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
