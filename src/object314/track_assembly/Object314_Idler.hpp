// Object314 idler subsystem.

#ifndef OBJECT314_IDLER_HPP
#define OBJECT314_IDLER_HPP

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Idler and tensioner model for the Object314 vehicle.
class CH_MODELS_API Object314_Idler : public ChTranslationalIdler {
  public:
    Object314_Idler(const std::string& name, VehicleSide side);
    ~Object314_Idler() {}

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector3d GetLocation(PointId which) override;

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector3d& GetCarrierInertia() override { return m_carrier_inertia; }
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const override { return m_carrier_radius; }

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const override { return 0; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetTensionerForceCallback() const override {
        return m_tensionerForceCB;
    }

    /// Return the free length for the tensioner spring.
    virtual double GetTensionerFreeLength() const override { return m_tensioner_l0; }

  private:
    VehicleSide m_side;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_tensionerForceCB;

    static const double m_carrier_mass;
    static const ChVector3d m_carrier_inertia;
    static const double m_carrier_radius;

    static const double m_tensioner_l0;
    static const double m_tensioner_k;
    static const double m_tensioner_c;
    static const double m_tensioner_f;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
