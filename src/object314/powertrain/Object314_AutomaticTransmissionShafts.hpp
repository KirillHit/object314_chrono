// Object314 automatic transmission model based on ChShaft objects.

#ifndef OBJECT314_AUTOMATIC_TRANSMISSION_SHAFTS_HPP
#define OBJECT314_AUTOMATIC_TRANSMISSION_SHAFTS_HPP

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Shafts-based powertrain model for the Object314 vehicle.
class CH_MODELS_API Object314_AutomaticTransmissionShafts : public ChAutomaticTransmissionShafts {
  public:
    Object314_AutomaticTransmissionShafts(const std::string& name);
    ~Object314_AutomaticTransmissionShafts() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetTransmissionBlockInertia() const override { return m_transmissionblock_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }
    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }

    virtual double GetUpshiftRPM() const override { return m_upshift_RPM; }
    virtual double GetDownshiftRPM() const override { return m_downshift_RPM; }

    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunctionInterp>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunctionInterp>& map) override;

  private:
    // Shaft inertias.
    static const double m_transmissionblock_inertia;
    static const double m_motorshaft_inertia;
    static const double m_driveshaft_inertia;
    static const double m_ingear_shaft_inertia;

    // Gear shifting characteristics
    static const double m_upshift_RPM;
    static const double m_downshift_RPM;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
