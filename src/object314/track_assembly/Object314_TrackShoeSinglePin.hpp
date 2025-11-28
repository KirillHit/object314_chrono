// Object314 track shoe subsystem (single pin).

#ifndef OBJECT314_TRACK_SHOE_SINGLE_PIN_HPP
#define OBJECT314_TRACK_SHOE_SINGLE_PIN_HPP

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Single-pin track shoe subsystem for the Object314 vehicle.
class CH_MODELS_API Object314_TrackShoeSinglePin : public ChTrackShoeSinglePin {
  public:
    Object314_TrackShoeSinglePin(const std::string& name);
    ~Object314_TrackShoeSinglePin() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return GuidePinType::CENTRAL_PIN; }
    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }
    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override { return m_shoe_pitch; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const override { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector3d& GetShoeInertia() const override { return m_shoe_inertia; }

    /// Return the location of the front contact cylinder.
    virtual double GetFrontCylinderLoc() const override { return m_front_cyl_loc; }
    /// Return the location of the rear contact cylinder.
    virtual double GetRearCylinderLoc() const override { return m_rear_cyl_loc; }
    /// Return the radius of the contact cylinders.
    virtual double GetCylinderRadius() const override { return m_cyl_radius; }

    /// Return the location of the guiding pin center, expressed in the shoe reference frame.
    virtual ChVector3d GetLateralContactPoint() const override { return m_pin_center; }

    /// Return contact geometry and material for interaction with terrain.
    virtual utils::ChBodyGeometry GetGroundContactGeometry() const override { return m_ground_geometry; }

  private:
    static const double m_shoe_height;
    static const double m_shoe_pitch;
    static const double m_shoe_mass;
    static const ChVector3d m_shoe_inertia;

    static const double m_cyl_radius;
    static const double m_front_cyl_loc;
    static const double m_rear_cyl_loc;

    static const ChVector3d m_pin_center;

    utils::ChBodyGeometry m_ground_geometry;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
