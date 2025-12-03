#include "chrono/output/ChOutputASCII.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "object314/Object314.hpp"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono/core/ChQuaternion.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::object314;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle position
ChVector3d initLoc(-4, 0, 0.5);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 1e-4;

// Use HHT + MKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system, double diff = 0);
void AddFallingObjects(ChSystem* system);

// =============================================================================
int main(int argc, char* argv[]) {
    // --------------------------
    // Construct the Object314 vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    ////TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    ////DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
    BrakeType brake_type = BrakeType::SIMPLE;
    EngineModelType engine_type = EngineModelType::SIMPLE;
    TransmissionModelType transmission_type = TransmissionModelType::AUTOMATIC_SIMPLE_MAP;

    Object314 object314;
    object314.SetContactMethod(contact_method);
    ////object314.SetTrackShoeType(shoe_type);
    ////object314.SetDrivelineType(driveline_type);
    object314.SetBrakeType(brake_type);
    object314.SetEngineType(engine_type);
    object314.SetTransmissionType(transmission_type);
    object314.SetChassisCollisionType(chassis_collision_type);

    ////object314.SetChassisFixed(true);
    ////object314.CreateTrack(false);

    // Control steering type (enable crossdrive capability)
    ////object314.GetDriveline()->SetGyrationMode(true);

    // Change collision detection system
    ////object314.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Change collision shape for road wheels, idlers, and rollers (true: cylinder; false: cylshell)
    ////object314.SetWheelCollisionType(false, false, false);

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

    // Disable gravity in this simulation
    ////object314.GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Change (SMC) contact force model
    ////if (contact_method == ChContactMethod::SMC) {
    ////static_cast<ChSystemSMC*>(object314.GetSystem())->SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    ////}

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.EnableCollision(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.EnableCollision(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor contacts involving one of the sprockets.
    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the chassis.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Render contact normals and/or contact forces.
    ////vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

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
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, -0.01, 0.04);
    terrain.GetMesh()->SetWireframe(true);
#endif

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    /* for (int idx = 0; idx < 20; ++idx)
        AddFixedObstacles(vehicle.GetSystem(), idx * 0.3); */
    ////AddFallingObjects(vehicle.GetSystem());

    // ------------------------
    // Create the driver system
    // ------------------------

    ChInteractiveDriver driver(vehicle);
    double steering_time = 1;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1;  // time to go from 0 to +1
    double braking_time = 1;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);
    driver.SetGains(2, 5, 5);
    driver.Initialize();

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // -----------------------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Object314 Vehicle Teleop");
            vis_irr->SetChaseCamera(trackPoint, 2.5, 0.2);
            vis_irr->SetChaseCameraAngle(1.59);
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional(60, -90);
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);
            vis_irr->AttachDriver(&driver);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Object314 Vehicle Teleop");
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetChaseCamera(trackPoint, 2.5, 0.75);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->AttachDriver(&driver);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "Object314";
    const std::string pov_dir = out_dir + "/POVRAY";
    const std::string img_dir = out_dir + "/IMG";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    /* if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    } */

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////vehicle.SetOutput(ChOutput::Type::ASCII, ChOutput::Mode::FRAMES, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            // Cannot use HHT + MKL with NSC contact
            use_mkl = false;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            break;
    }

#ifndef CHRONO_PARDISO_MKL
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        std::cout << "Solver: PardisoMKL" << std::endl;
        std::cout << "Integrator: HHT" << std::endl;

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        object314.GetSystem()->SetSolver(mkl_solver);

        object314.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(object314.GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxIters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetStepControl(false);
        integrator->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
        ////integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        object314.GetSystem()->SetSolver(solver);

        object314.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    }

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

        // Current driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        object314.Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
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