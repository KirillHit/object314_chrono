// Object314 chassis subsystem.

#include <cmath>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_Chassis.hpp"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Object314_Chassis::m_body_mass = 127.93;  // base_link
const ChVector3d Object314_Chassis::m_body_inertiaXX(1.3597, 4.5352, 5.3896);
const ChVector3d Object314_Chassis::m_body_inertiaXY(-4.8039e-06, 0.00010341, 0.016572);
const ChVector3d Object314_Chassis::m_body_COM_loc(0.038003, -0.00046893, -0.092409);
const ChCoordsys<> Object314_Chassis::m_driverCsys(ChVector3d(0.42865,
                                                              0.038083,
                                                              -0.00585),  // front camera support offset
                                                   ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Object314_Chassis::Object314_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single material with default properties.
    ChContactMaterialData minfo;
    m_geometry.materials.push_back(minfo);

    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    double length = 1.172;
    double width = 0.584;
    double height = 0.45;

    ChVector3d dims(length, width, height);
    ChVector3d loc(0.0, 0.0, height / 2.0);
    ChQuaternion<> rot(1, 0, 0, 0);
    utils::ChBodyGeometry::BoxShape box(loc, rot, dims);

    m_geometry.vis_boxes.push_back(box);

    m_geometry.vis_model_file = GetVehicleDataFile("object314/Hull.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box.matID = 0;
            m_geometry.coll_boxes.push_back(box);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("object314/Hull.obj"), 0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
