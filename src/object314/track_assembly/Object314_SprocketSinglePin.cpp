// Object314 sprocket subsystem (single pin).

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "Object314_SprocketSinglePin.hpp"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace object314 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int Object314_SprocketSinglePin::m_num_teeth = 12;

const double Object314_SprocketSinglePin::m_gear_mass = 1.19;
const ChVector3d Object314_SprocketSinglePin::m_gear_inertia(0.00339, 0.00463, 0.00339);
const double Object314_SprocketSinglePin::m_axle_inertia = 0.0021;
const double Object314_SprocketSinglePin::m_separation = 0.0;

const double Object314_SprocketSinglePin::m_gear_RT = 0.21 / 2;           // Outer radius
const double Object314_SprocketSinglePin::m_gear_RC = (0.21 + 0.18) / 4;  // Arc centers radius
const double Object314_SprocketSinglePin::m_gear_R = 0.07;                // Arc radius
const double Object314_SprocketSinglePin::m_gear_RA = 0.18 / 2;           // Assembly radius

const double Object314_SprocketSinglePin::m_lateral_backlash = 0.005;

const std::string Object314_SprocketSinglePinLeft::m_meshFile = "object314/Sprocket_L.obj";
const std::string Object314_SprocketSinglePinRight::m_meshFile = "object314/Sprocket_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Object314_SprocketSinglePin::Object314_SprocketSinglePin(const std::string& name) : ChSprocketSinglePin(name) {}

void Object314_SprocketSinglePin::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e8f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Object314_SprocketSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        // auto trimesh = CreateVisualizationMesh(0.15, 0.03, 0.01);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetTexture(GetChronoDataFile("textures/metal.jpg"), 20, 20);
        trimesh_shape->SetMutable(false);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // namespace object314
}  // end namespace vehicle
}  // end namespace chrono
