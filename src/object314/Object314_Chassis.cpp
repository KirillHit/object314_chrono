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
const double Object314_Chassis::m_body_mass = 25000.00;
const ChVector3d Object314_Chassis::m_body_inertiaXX(13653.38542, 98182.70833, 101954.4271);
const ChVector3d Object314_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d Object314_Chassis::m_body_COM_loc(-2.7958, 0, 0.0);
const ChCoordsys<> Object314_Chassis::m_driverCsys(ChVector3d(0.0, 0.5, 0.0), ChQuaternion<>(1, 0, 0, 0));

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

    // Belly shape (all dimensions in cm)
    //   width: 170
    //   points in x-z transversal plane: (-417.0 -14.3), (4.1, -14.3), (21.4, 34.3)
    //   thickness: 20
    double width = 2.045;
    double Ax = -5.45;
    double Az = -0.4;
    double Bx = 0.041;
    double Bz = -0.243;
    double Cx = 0.214;
    double Cz = 0.343;
    double thickness = 0.2;

    ChVector3d dims1((Bx - Ax), width, thickness);
    ChVector3d loc1(0.5 * (Ax + Bx), 0.0, Az + 0.5 * thickness);
    ChQuaternion<> rot1(1, 0, 0, 0);
    utils::ChBodyGeometry::BoxShape box1(loc1, rot1, dims1);

    double alpha = std::atan2(Cz - Bz, Cx - Bx);  // pitch angle of front box

    ChVector3d dims2((Cx - Bx) / std::cos(alpha), width, thickness);
    ChVector3d loc2(0.5 * (Bx + Cx) - 0.5 * thickness * std::sin(alpha), 0.0,
                    0.5 * (Bz + Cz) + 0.5 * thickness * std::cos(alpha));
    ChQuaternion<> rot2 = QuatFromAngleY(-alpha);
    utils::ChBodyGeometry::BoxShape box2(loc2, rot2, dims2);

    m_geometry.vis_boxes.push_back(box1);
    m_geometry.vis_boxes.push_back(box2);

    m_geometry.vis_model_file = GetVehicleDataFile("object314/Hull.stl");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            box2.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            m_geometry.coll_boxes.push_back(box2);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("object314/Hull.stl"), 0);
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
