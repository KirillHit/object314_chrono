// Object314 suspension subsystem.

#include "Object314_Suspension.hpp"
#include "Object314_RoadWheel.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Object314_Suspension::m_arm_mass = 1.0;
const ChVector3d Object314_Suspension::m_arm_inertia(0.0049, 0.0102, 0.0102);
const double Object314_Suspension::m_arm_radius = 0.02;

const double Object314_Suspension::m_torsion_a0 = 0;
const double Object314_Suspension::m_torsion_k = 1010.0;
const double Object314_Suspension::m_torsion_c = 6.0;
const double Object314_Suspension::m_torsion_t = 0.0;

const double Object314_Suspension::m_shock_c = 213.0;

// -----------------------------------------------------------------------------
// Object314 spring functor class - implements a (non)linear rotational spring
// -----------------------------------------------------------------------------
class Object314_SpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    Object314_SpringTorque(double k, double c, double t) : m_k(k), m_c(c), m_t(t) {}

    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override {
        return m_t - m_k * angle - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_t;
};

// -----------------------------------------------------------------------------
// Object314 shock functor class - implements a (non)linear translational damper
// -----------------------------------------------------------------------------
class Object314_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    Object314_ShockForce(double c) : m_c(c) {}

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Object314_Suspension::Object314_Suspension(const std::string& name, VehicleSide side, int index, bool has_shock)
    : ChTranslationalDamperSuspension(name, has_shock), m_side(side) {
    // Instantiate the force callback for the shock (damper).
    m_shock_forceCB = chrono_types::make_shared<Object314_ShockForce>(m_shock_c);

    // Instantiate the torque callback for the spring.
    m_spring_torqueCB = chrono_types::make_shared<Object314_SpringTorque>(m_torsion_k, m_torsion_c, m_torsion_t);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = chrono_types::make_shared<Object314_RoadWheelLeft>(index);
    else
        m_road_wheel = chrono_types::make_shared<Object314_RoadWheelRight>(index);
}

Object314_Suspension::~Object314_Suspension() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector3d Object314_Suspension::GetLocation(PointId which) {
    ChVector3d point;

    switch (which) {
        case ARM:
            point = ChVector3d(0.1, 0.0, 0.11);
            break;
        case ARM_WHEEL:
            point = ChVector3d(0.0, 0.0, 0.0);
            break;
        case ARM_CHASSIS:
            point = ChVector3d(0.10, -0.13, 0.11);
            break;
        case SHOCK_A:
            point = ChVector3d(0.0, 0.0, 0.0);
            break;
        case SHOCK_C:
            point = ChVector3d(-0.05, 0.0, 0.1);
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
