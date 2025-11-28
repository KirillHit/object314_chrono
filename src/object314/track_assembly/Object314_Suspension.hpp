// Object314 suspension subsystem.

#ifndef OBJECT314_SUSPENSION_HPP
#define OBJECT314_SUSPENSION_HPP

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChTranslationalDamperSuspension.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Linear-damper Object314 track suspension.
class CH_MODELS_API Object314_Suspension : public ChTranslationalDamperSuspension {
  public:
    Object314_Suspension(const std::string& name, VehicleSide side, int index, bool has_shock);
    ~Object314_Suspension();

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector3d GetLocation(PointId which) override;

    /// Return the mass of the arm body.
    virtual double GetArmMass() const override { return m_arm_mass; }
    /// Return the moments of inertia of the arm body.
    virtual const ChVector3d& GetArmInertia() const override { return m_arm_inertia; }
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const override { return m_arm_radius; }

    /// Return the free (rest) angle of the spring element.
    virtual double GetSpringRestAngle() const override { return 0; }
    /// Return the functor object for the torsional spring torque.
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> GetSpringTorqueFunctor() const override {
        return m_spring_torqueCB;
    }
    /// Return the functor object for the translational shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetShockForceFunctor() const override { return m_shock_forceCB; }

  private:
    VehicleSide m_side;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_spring_torqueCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shock_forceCB;

    static const double m_arm_mass;
    static const ChVector3d m_arm_inertia;
    static const double m_arm_radius;

    static const double m_torsion_a0;
    static const double m_torsion_k;
    static const double m_torsion_c;
    static const double m_torsion_t;

    static const double m_shock_c;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
