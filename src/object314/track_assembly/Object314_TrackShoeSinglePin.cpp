// Object314 track shoe subsystem (single pin).

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_TrackShoeSinglePin.hpp"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Object314_TrackShoeSinglePin::m_shoe_height = 0.0065;
const double Object314_TrackShoeSinglePin::m_shoe_pitch = 0.0505;

const double Object314_TrackShoeSinglePin::m_shoe_mass = 0.87;
const ChVector3d Object314_TrackShoeSinglePin::m_shoe_inertia(0.0035, 0.00019, 0.0030);

const double Object314_TrackShoeSinglePin::m_cyl_radius = 0.00325;
const double Object314_TrackShoeSinglePin::m_front_cyl_loc = 0.0125;
const double Object314_TrackShoeSinglePin::m_rear_cyl_loc = -0.0125;

const ChVector3d Object314_TrackShoeSinglePin::m_pin_center(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------

Object314_TrackShoeSinglePin::Object314_TrackShoeSinglePin(const std::string& name) : ChTrackShoeSinglePin(name) {
    // Contact materials

    // Material for cylindrical surfaces (sprocket contact)
    m_shoe_sprk_minfo.mu = 0.8f;
    m_shoe_sprk_minfo.cr = 0.75f;
    m_shoe_sprk_minfo.Y = 1e8f;

    // Material 0: pad bottom (ground contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.materials.push_back(minfo);
    }

    // Material 1: guide pin (wheel contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.materials.push_back(minfo);
    }

    // Geometry

    // Collision box: pad bottom (ground contact)
    utils::ChBodyGeometry::BoxShape box_bottom(ChVector3d(0, 0, 0.0), QUNIT, ChVector3d(0.0505, 0.22, 0.0065), 0);

    // Collision box: guide pin (wheel contact)
    utils::ChBodyGeometry::BoxShape box_guide(ChVector3d(0, 0, 0.0), QUNIT, ChVector3d(0.018, 0.088, 0.025), 1);

    m_geometry.coll_boxes.push_back(box_bottom);
    m_geometry.coll_boxes.push_back(box_guide);

    m_ground_geometry.materials = m_geometry.materials;
    m_ground_geometry.coll_boxes.push_back(box_bottom);

    m_geometry.vis_boxes.push_back(box_bottom);
    m_geometry.vis_boxes.push_back(box_guide);

    m_geometry.vis_model_file = GetVehicleDataFile("object314/TrackShoeSinglePin.obj");
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
