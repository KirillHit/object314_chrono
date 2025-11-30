// Object314 idler subsystem.

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_Idler.hpp"
#include "Object314_IdlerWheel.hpp"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Object314_Idler::m_carrier_mass = 0.5;
const ChVector3d Object314_Idler::m_carrier_inertia(0.002, 0.002, 0.002);
const double Object314_Idler::m_carrier_radius = 0.0001;

const double Object314_Idler::m_tensioner_l0 = 0.36;
const double Object314_Idler::m_tensioner_f = 9.81 * 300.0 / 10.0;  // 10% Weight Force      M113: 2e4;
const double Object314_Idler::m_tensioner_k = 5e5;
const double Object314_Idler::m_tensioner_c = Object314_Idler::m_tensioner_k * 0.05;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class Object314_TensionerForce : public ChLinkTSDA::ForceFunctor {
  public:
    Object314_TensionerForce(double k, double c, double f) : m_k(k), m_c(c), m_f(f) {}

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return m_f - m_k * (length - rest_length) - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_f;
};

Object314_Idler::Object314_Idler(const std::string& name, VehicleSide side) : ChTranslationalIdler(name), m_side(side) {
    m_tensionerForceCB =
        chrono_types::make_shared<Object314_TensionerForce>(m_tensioner_k, m_tensioner_c, m_tensioner_f);

    // Create the associated idler wheel.
    if (side == LEFT)
        m_idler_wheel = chrono_types::make_shared<Object314_IdlerWheelLeft>();
    else
        m_idler_wheel = chrono_types::make_shared<Object314_IdlerWheelRight>();
}

const ChVector3d Object314_Idler::GetLocation(PointId which) {
    ChVector3d point;

    switch (which) {
        case CARRIER_WHEEL:
            point = ChVector3d(0, 0, 0);
            break;
        case CARRIER:
            point = ChVector3d(0, -0.05, 0);
            break;
        case CARRIER_CHASSIS:
            point = ChVector3d(0, -0.1, 0);
            break;
        case TSDA_CARRIER:
            point = ChVector3d(0, -0.13, 0);
            break;
        case TSDA_CHASSIS:
            point = ChVector3d(-0.36, -0.13, 0);
            break;
        default:
            point = ChVector3d(0, 0, 0);
            break;
    }

    if (m_side == RIGHT)
        point.y() *= -1;

    return point;
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
