// A simplified Object314 driveline.

#ifndef OBJECT314_SIMPLE_DRIVELINE_HPP
#define OBJECT314_SIMPLE_DRIVELINE_HPP

#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace object314 {

/// Simple driveline model for the Object314 vehicle (purely kinematic).
class CH_MODELS_API Object314_SimpleDriveline : public ChSimpleTrackDriveline {
  public:
    Object314_SimpleDriveline();
    ~Object314_SimpleDriveline() {}

    /// Return the torque bias ratio for the differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetDifferentialMaxBias() const override { return m_diff_maxBias; }

  private:
    static const double m_diff_maxBias;
};

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono

#endif
