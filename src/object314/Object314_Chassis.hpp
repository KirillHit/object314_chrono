// Object314 chassis subsystem.

#ifndef Object314_CHASSIS_HPP
#define Object314_CHASSIS_HPP

#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 chassis subsystem.
class CH_MODELS_API Object314_Chassis : public ChRigidChassis {
  public:
    Object314_Chassis(const std::string& name,
                      bool fixed = false,
                      CollisionType chassis_collision_type = CollisionType::NONE);
    ~Object314_Chassis() {}

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(m_body_COM_loc, QUNIT); }

    ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const ChVector3d m_body_inertiaXX;
    static const ChVector3d m_body_inertiaXY;
    static const ChVector3d m_body_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
