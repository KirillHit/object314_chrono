#include "chrono/input_output/ChOutputASCII.h"
#include "chrono/input_output/ChUtilsInputOutput.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "object314/Object314.hpp"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono/core/ChQuaternion.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <algorithm>
#include <iostream>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::object314;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Initial vehicle position
ChVector3d initLoc(-4, 0, 0.5);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 1e-4;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// Direct keyboard torque control [N m].
double keyboard_track_torque_limit = 100.0;
double keyboard_track_torque_step = 1.0;

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system, double diff = 0);
void AddFallingObjects(ChSystem* system);

// =============================================================================

struct TrackTorqueCommand {
    double left = 0;
    double right = 0;
};

class TrackTorqueKeyboardHandler : public irr::IEventReceiver {
  public:
    TrackTorqueKeyboardHandler(TrackTorqueCommand& command, double torque_step, double torque_limit)
        : m_command(command), m_torque_step(torque_step), m_torque_limit(torque_limit) {}

    bool OnEvent(const irr::SEvent& event) override {
        if (event.EventType != irr::EET_KEY_INPUT_EVENT || !event.KeyInput.PressedDown) {
            return false;
        }

        switch (event.KeyInput.Key) {
            case irr::KEY_KEY_Q:
                Adjust(m_torque_step, 0);
                return true;
            case irr::KEY_KEY_A:
                Adjust(-m_torque_step, 0);
                return true;
            case irr::KEY_KEY_E:
                Adjust(0, m_torque_step);
                return true;
            case irr::KEY_KEY_D:
                Adjust(0, -m_torque_step);
                return true;
            case irr::KEY_KEY_W:
                Adjust(m_torque_step, m_torque_step);
                return true;
            case irr::KEY_KEY_S:
                Adjust(-m_torque_step, -m_torque_step);
                return true;
            case irr::KEY_SPACE:
                m_command.left = 0;
                m_command.right = 0;
                std::cout << "Track torques: L=0 N m, R=0 N m" << std::endl;
                return true;
            default:
                return false;
        }
    }

  private:
    void Adjust(double left_delta, double right_delta) {
        m_command.left = std::clamp(m_command.left + left_delta, -m_torque_limit, m_torque_limit);
        m_command.right = std::clamp(m_command.right + right_delta, -m_torque_limit, m_torque_limit);
        std::cout << "Track torques: L=" << m_command.left << " N m, R=" << m_command.right << " N m" << std::endl;
    }

    TrackTorqueCommand& m_command;
    double m_torque_step;
    double m_torque_limit;
};

// =============================================================================
int main(int argc, char* argv[]) {
    // --------------------------
    // Construct the Object314 vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;

    Object314 object314;
    object314.SetContactMethod(contact_method);
    object314.SetChassisCollisionType(chassis_collision_type);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    object314.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    object314.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis = VisualizationType::MESH;
    object314.SetChassisVisualizationType(VisualizationType::MESH);
    object314.SetSprocketVisualizationType(track_vis);
    object314.SetIdlerVisualizationType(track_vis);
    object314.SetRollerVisualizationType(track_vis);
    object314.SetSuspensionVisualizationType(track_vis);
    object314.SetIdlerWheelVisualizationType(track_vis);
    object314.SetRoadWheelVisualizationType(track_vis);
    object314.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    auto& vehicle = object314.GetVehicle();

    // Monitor contacts involving one of the sprockets.
    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // ------------------
    // Create the terrain
    // ------------------
#ifndef SMC_TERRAIN
    RigidTerrain terrain(object314.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.75f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 20, 20);
    terrain.Initialize();
#else
    // Create the SCM deformable terrain
    vehicle::SCMTerrain terrain(object314.GetSystem());

    // Displace/rotate the terrain reference frame.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain frame by -90 degrees about the X axis.
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(0, -0.5, 0), QuatFromAngleX(0)));

    // Use a regular grid
    double length = 14;
    double width = 14;
    double mesh_resolution = 0.02;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    terrain.SetSoilParameters(0.82e6,   // Bekker Kphi
                              0.14e4,   // Bekker Kc
                              1.0,      // Bekker n exponent
                              0.017e4,  // Mohr cohesive limit (Pa)
                              35,       // Mohr friction limit (degrees)
                              1.78e-2,  // Janosi shear coefficient (m)
                              2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                              3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Enable/disable bulldozing effects
    bool enable_bulldozing = true;

    // Set up bulldozing factors
    terrain.EnableBulldozing(enable_bulldozing);
    terrain.SetBulldozingParameters(55,  // angle of friction for erosion of displaced material at the border of the rut
                                    1,   // displaced material vs downward pressed material.
                                    5,   // number of erosion refinements per timestep
                                    6);  // number of concentric vertex selections subject to erosion

    // Set some visualization parameters
    terrain.SetColormap(ChColormap::Type::COPPER);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, -0.01, 0.04);
    terrain.GetMesh()->SetWireframe(true);
#endif

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    // ---------------------------
    // Create the keyboard controls
    // ---------------------------

    TrackTorqueCommand track_torque_command;
    std::cout << "\nKeyboard torque control:" << std::endl;
    std::cout << "  Q/A: left track torque +/- " << keyboard_track_torque_step << " N m" << std::endl;
    std::cout << "  E/D: right track torque +/- " << keyboard_track_torque_step << " N m" << std::endl;
    std::cout << "  W/S: both track torques +/- " << keyboard_track_torque_step << " N m" << std::endl;
    std::cout << "  Space: zero both track torques" << std::endl;
    std::cout << "  Range: [" << -keyboard_track_torque_limit << ", " << keyboard_track_torque_limit << "] N m"
              << std::endl;

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // -----------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    std::shared_ptr<TrackTorqueKeyboardHandler> torque_keyboard;
    vis->SetWindowTitle("Object314 Vehicle Teleop");
    vis->EnableStats(false);
    vis->SetChaseCamera(trackPoint, 2.5, 0.2);
    vis->SetChaseCameraAngle(1.59);
    vis->SetChaseCameraMultipliers(1e-4, 10);
    vis->Initialize();
    vis->AddLightDirectional(60, -90);
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);
    torque_keyboard = chrono_types::make_shared<TrackTorqueKeyboardHandler>(
        track_torque_command, keyboard_track_torque_step, keyboard_track_torque_limit);
    vis->AddUserEventReceiver(torque_keyboard.get());

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "Object314";
    const std::string pov_dir = out_dir + "/POVRAY";
    const std::string img_dir = out_dir + "/IMG";

    auto out_path = filesystem::path(out_dir);
    if (!out_path.is_directory() && !filesystem::create_directory(out_path)) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    /* if (povray_output) {
        auto pov_path = filesystem::path(pov_dir);
        if (!pov_path.is_directory() && !filesystem::create_directory(pov_path)) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    } */

    if (img_output) {
        auto img_path = filesystem::path(img_dir);
        if (!img_path.is_directory() && !filesystem::create_directory(img_path)) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            break;
    }

    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(120);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    object314.GetSystem()->SetSolver(solver);

    object314.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "\n============ Vehicle subsystems ============" << std::endl;
    vehicle.LogSubsystemTypes();

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = vehicle.GetTrackAssembly(LEFT);
            auto track_R = vehicle.GetTrackAssembly(RIGHT);
            cout << "Time: " << object314.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << object314.GetSystem()->GetNumContacts() << endl;
            const ChFrameMoving<>& c_ref = object314.GetChassisBody()->GetFrameRefToAbs();
            const ChVector3d& c_pos = vehicle.GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector3d& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector3d& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumTrackSuspensions(); i++) {
                cout << " " << track_L->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumTrackSuspensions(); i++) {
                cout << " " << track_R->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            // Zero-pad frame numbers in file names for postprocessing
            if (povray_output) {
                std::ostringstream filename;
                filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                utils::WriteVisualizationAssets(object314.GetSystem(), filename.str());
            }
            if (img_output && step_number > 200) {
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        DriverInputs vehicle_inputs = {};

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        terrain.Synchronize(time);
        object314.Synchronize(time, vehicle_inputs);
        object314.ApplyTrackTorques(track_torque_command.left, track_torque_command.right);
        vis->Synchronize(time, vehicle_inputs);

        // Advance simulation for one timestep for all modules
        terrain.Advance(step_size);
        object314.Advance(step_size);
        vis->Advance(step_size);

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    vehicle.WriteContacts("object314_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system, double diff) {
    double radius = 0.2;
    double length = 6;

    auto obstacle = chrono_types::make_shared<ChBody>();
    obstacle->SetPos(ChVector3d(-3 + diff, 0, -0.18));
    obstacle->SetFixed(true);
    obstacle->EnableCollision(true);

    // Visualization
    auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
    vis_shape->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 10, 10);
    obstacle->AddVisualShape(vis_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Contact
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(obst_mat, radius, length);
    obstacle->AddCollisionShape(ct_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    system->AddBody(obstacle);
}

// =============================================================================
void AddFallingObjects(ChSystem* system) {
    double radius = 0.1;
    double mass = 10;

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
    ball->SetPos(initLoc + ChVector3d(-3, 0, 2));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetPosDt(ChVector3d(3, 0, 0));
    ball->SetFixed(false);

    ChContactMaterialData minfo;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    ball->EnableCollision(true);
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeSphere>(obst_mat, radius);
    ball->AddCollisionShape(ct_shape);

    auto vis_shape = chrono_types::make_shared<ChVisualShapeSphere>(radius);
    vis_shape->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(vis_shape);

    system->AddBody(ball);
}
