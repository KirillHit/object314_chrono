// Object314 sprocket subsystem (single pin).

#ifndef OBJECT314_SPROCKET_SINGLE_PIN_HPP
#define OBJECT314_SPROCKET_SINGLE_PIN_HPP

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Object314 sprocket subsystem, suitable for interaction with single-pin track shoes (base class).
class CH_MODELS_API Object314_SprocketSinglePin : public ChSprocketSinglePin {
  public:
    virtual ~Object314_SprocketSinglePin() {}

    /// Get the number of teeth of the gear.
    virtual unsigned int GetNumTeeth() const override { return m_num_teeth; }

    /// Get the radius of the gear.
    /// This quantity is used during the automatic track assembly.
    virtual double GetAssemblyRadius() const override { return m_gear_RA; }

    /// Get the addendum radius.
    /// This quantity is an average radius for sprocket-track engagement used to estimate longitudinal slip.
    virtual double GetAddendumRadius() const override { return m_gear_RT; }

    /// Return the mass of the gear body.
    virtual double GetGearMass() const override { return m_gear_mass; }
    /// Return the moments of inertia of the gear body.
    virtual const ChVector3d& GetGearInertia() override { return m_gear_inertia; }
    /// Return the inertia of the axle shaft.
    virtual double GetAxleInertia() const override { return m_axle_inertia; }
    /// Return the distance between the two gear profiles.
    virtual double GetSeparation() const override { return m_separation; }

    /// Return the radius of the addendum circle.
    virtual double GetOuterRadius() const override { return m_gear_RT; }
    /// Return the radius of the (concave) tooth circular arc.
    virtual double GetArcRadius() const override { return m_gear_R; }
    /// Return the radius of the tooth arc centers.
    virtual double GetArcCentersRadius() const override { return m_gear_RC; }

    /// Return the allowed backlash (play) before lateral contact with track shoes is enabled (to prevent detracking).
    virtual double GetLateralBacklash() const override { return m_lateral_backlash; }

  protected:
    Object314_SprocketSinglePin(const std::string& name);

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization of the sprocket.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    virtual std::string GetMeshFile() const = 0;

    static const int m_num_teeth;

    static const double m_gear_mass;
    static const ChVector3d m_gear_inertia;
    static const double m_axle_inertia;
    static const double m_separation;

    // Gear profile data
    static const double m_gear_RT;
    static const double m_gear_RC;
    static const double m_gear_R;
    static const double m_gear_RA;

    static const double m_lateral_backlash;
};

/// M113 sprocket subsystem, suitable for interaction with single-pin track shoes (left side).
class CH_MODELS_API Object314_SprocketSinglePinLeft : public Object314_SprocketSinglePin {
  public:
    Object314_SprocketSinglePinLeft() : Object314_SprocketSinglePin("Object314_SprocketLeft") {}
    ~Object314_SprocketSinglePinLeft() {}

    virtual std::string GetMeshFile() const override { return GetVehicleDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// M113 sprocket subsystem, suitable for interaction with single-pin track shoes (right side).
class CH_MODELS_API Object314_SprocketSinglePinRight : public Object314_SprocketSinglePin {
  public:
    Object314_SprocketSinglePinRight() : Object314_SprocketSinglePin("Object314_SprocketRight") {}
    ~Object314_SprocketSinglePinRight() {}

    virtual std::string GetMeshFile() const override { return GetVehicleDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
