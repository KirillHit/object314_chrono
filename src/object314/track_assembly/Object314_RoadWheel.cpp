// Object314 road wheel subsystem.

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_RoadWheel.hpp"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Object314_RoadWheel::m_wheel_mass = 1.0698;
const ChVector3d Object314_RoadWheel::m_wheel_inertia(0.0027827, 0.0051995, 0.0027827);
const double Object314_RoadWheel::m_wheel_radius = 0.19 / 2;
const double Object314_RoadWheel::m_wheel_width = 0.1;
const double Object314_RoadWheel::m_wheel_gap = 0.21;

const std::string Object314_RoadWheelLeft::m_meshFile = "object314/Roller.obj";
const std::string Object314_RoadWheelRight::m_meshFile = "object314/Roller.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Object314_RoadWheel::Object314_RoadWheel(const std::string& name) : ChDoubleTrackWheel(name) {}

void Object314_RoadWheel::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Object314_RoadWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_wheel->AddVisualShape(trimesh_shape);
    } else {
        ChDoubleTrackWheel::AddVisualizationAssets(vis);
    }
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
