// Object314 road wheel subsystem.

#ifndef OBJECT314_ROAD_WHEEL_HPP
#define OBJECT314_ROAD_WHEEL_HPP

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Road-wheel model for the Object314 vehicle (base class).
class CH_MODELS_API Object314_RoadWheel : public ChDoubleTrackWheel {
  public:
    virtual ~Object314_RoadWheel() {}

    /// Return the mass of the idler wheel body.
    virtual double GetMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const ChVector3d& GetInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetGap() const override { return m_wheel_gap; }

  protected:
    Object314_RoadWheel(const std::string& name);

    virtual VehicleSide GetVehicleSide() const = 0;

    virtual std::string GetMeshFile() const = 0;

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization of the road wheel.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    static const double m_wheel_mass;
    static const ChVector3d m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;
};

/// Road-wheel model for the M113 vehicle (left side).
class CH_MODELS_API Object314_RoadWheelLeft : public Object314_RoadWheel {
  public:
    Object314_RoadWheelLeft(int index) : Object314_RoadWheel("Object314_RoadWheelLeft_" + std::to_string(index)) {}
    ~Object314_RoadWheelLeft() {}

    virtual VehicleSide GetVehicleSide() const override { return LEFT; }

    virtual std::string GetMeshFile() const override { return GetVehicleDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// Road-wheel model for the M113 vehicle (right side).
class CH_MODELS_API Object314_RoadWheelRight : public Object314_RoadWheel {
  public:
    Object314_RoadWheelRight(int index) : Object314_RoadWheel("Object314_RoadWheelRight_" + std::to_string(index)) {}
    ~Object314_RoadWheelRight() {}

    virtual VehicleSide GetVehicleSide() const override { return RIGHT; }

    virtual std::string GetMeshFile() const override { return GetVehicleDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
