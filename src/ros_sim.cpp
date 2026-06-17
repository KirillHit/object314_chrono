#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/collision/ChCollisionShapeSphere.h"
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "object314/Object314.hpp"

#include "rclcpp/rclcpp.hpp"
#include "vehicle_nmpc_interfaces/srv/reset_simulation.hpp"
#include "vehicle_nmpc_interfaces/srv/step_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::object314;

using vehicle_nmpc_interfaces::srv::ResetSimulation;
using vehicle_nmpc_interfaces::srv::StepSimulation;

namespace {

constexpr double kInitialZ = 0.45;
constexpr double kTerrainLength = 100.0;
constexpr double kTerrainWidth = 100.0;
constexpr double kInternalStepSize = 1e-4;
constexpr double kMaxStepRequest = 1.0;
constexpr double kMaxTrackTorque = 250.0;
constexpr double kDefaultTerrainFriction = 0.9;
constexpr double kDefaultTerrainRestitution = 0.75;
constexpr double kDefaultTerrainYoungModulus = 2e7;
constexpr double kDefaultTopViewHeight = 10.0;
constexpr double kRenderStepSize = 1.0 / 60.0;
constexpr bool kEnableSmcTerrain = true;
constexpr bool kEnableFixedObstacles = false;
constexpr double kScmTerrainLength = 14.0;
constexpr double kScmTerrainWidth = 14.0;
constexpr double kScmMeshResolution = 0.02;
constexpr bool kEnableScmBulldozing = true;
constexpr double kObstacleRadius = 0.2;
constexpr double kObstacleCenterZ = -0.18;
constexpr double kObstacleGridStartX = 1.0;
constexpr double kObstacleGridCenterY = 0.0;
constexpr double kObstacleGridSpacingX = 1.0;
constexpr double kObstacleGridSpacingY = 1.0;
constexpr int kObstacleGridSizeX = 5;
constexpr int kObstacleGridSizeY = 5;
const std::string kDefaultRosNamespace = "/vehicle";

struct PlantState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
};

PlantState state_from_vector(const std::vector<double>& values) {
    PlantState state;
    if (values.size() >= 6) {
        state.x = values[0];
        state.y = values[1];
        state.yaw = values[2];
        state.vx = values[3];
        state.vy = values[4];
        state.yaw_rate = values[5];
    }
    return state;
}

std::vector<double> state_to_vector(const PlantState& state) {
    return {state.x, state.y, state.yaw, state.vx, state.vy, state.yaw_rate};
}

std::unique_ptr<ChTerrain> create_terrain(ChSystem* system) {
    if constexpr (kEnableSmcTerrain) {
        auto terrain = std::make_unique<vehicle::SCMTerrain>(system);
        terrain->SetReferenceFrame(ChCoordsys<>(ChVector3d(0, -0.5, 0), QuatFromAngleX(0)));
        terrain->Initialize(kScmTerrainLength, kScmTerrainWidth, kScmMeshResolution);
        terrain->SetSoilParameters(0.82e6,   // Bekker Kphi
                                   0.14e4,   // Bekker Kc
                                   1.0,      // Bekker n exponent
                                   0.017e4,  // Mohr cohesive limit (Pa)
                                   35,       // Mohr friction limit (degrees)
                                   1.78e-2,  // Janosi shear coefficient (m)
                                   2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                   3e4       // Damping (Pa s/m), proportional to negative vertical speed
        );
        terrain->EnableBulldozing(kEnableScmBulldozing);
        terrain->SetBulldozingParameters(55, 1, 5, 6);
        terrain->SetColormap(ChColormap::Type::COPPER);
        terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, -0.01, 0.04);
        terrain->GetMesh()->SetWireframe(true);
        return terrain;
    } else {
        auto terrain = std::make_unique<RigidTerrain>(system);
        ChContactMaterialData minfo;
        minfo.mu = kDefaultTerrainFriction;
        minfo.cr = kDefaultTerrainRestitution;
        minfo.Y = kDefaultTerrainYoungModulus;
        auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
        auto patch = terrain->AddPatch(patch_mat, CSYSNORM, kTerrainLength, kTerrainWidth);
        patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
        patch->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 20, 20);
        terrain->Initialize();
        return terrain;
    }
}

void add_fixed_obstacles(ChSystem* system) {
    ChContactMaterialData minfo;
    minfo.mu = kDefaultTerrainFriction;
    minfo.cr = 0.01f;
    minfo.Y = kDefaultTerrainYoungModulus;
    auto obstacle_mat = minfo.CreateMaterial(system->GetContactMethod());
    const double first_y = kObstacleGridCenterY - 0.5 * kObstacleGridSpacingY * (kObstacleGridSizeY - 1);

    for (int ix = 0; ix < kObstacleGridSizeX; ++ix) {
        for (int iy = 0; iy < kObstacleGridSizeY; ++iy) {
            auto obstacle = chrono_types::make_shared<ChBody>();
            obstacle->SetPos(ChVector3d(kObstacleGridStartX + ix * kObstacleGridSpacingX,
                                        first_y + iy * kObstacleGridSpacingY, kObstacleCenterZ));
            obstacle->SetFixed(true);
            obstacle->EnableCollision(true);

            auto vis_shape = chrono_types::make_shared<ChVisualShapeSphere>(kObstacleRadius);
            vis_shape->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"));
            obstacle->AddVisualShape(vis_shape);

            auto collision_shape = chrono_types::make_shared<ChCollisionShapeSphere>(obstacle_mat, kObstacleRadius);
            obstacle->AddCollisionShape(collision_shape);

            system->AddBody(obstacle);
        }
    }
}

double yaw_from_quaternion(const ChQuaternion<>& q) {
    const double sin_yaw = 2.0 * (q.e0() * q.e3() + q.e1() * q.e2());
    const double cos_yaw = 1.0 - 2.0 * (q.e2() * q.e2() + q.e3() * q.e3());
    return std::atan2(sin_yaw, cos_yaw);
}

double wrap_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

class Object314RosPlant {
  public:
    Object314RosPlant() { reset(PlantState{}); }

    PlantState reset(const PlantState& requested_state) {
        if (vis_) {
            vis_->GetSystems().clear();
        }

        object314_ = std::make_unique<Object314>();
        object314_->SetContactMethod(ChContactMethod::SMC);
        object314_->SetChassisCollisionType(CollisionType::NONE);
        object314_->SetInitPosition(ChCoordsys<>(ChVector3d(requested_state.x, requested_state.y, kInitialZ),
                                                 QuatFromAngleZ(requested_state.yaw)));
        object314_->Initialize();

        object314_->SetChassisVisualizationType(VisualizationType::MESH);
        object314_->SetSprocketVisualizationType(VisualizationType::MESH);
        object314_->SetIdlerVisualizationType(VisualizationType::MESH);
        object314_->SetRollerVisualizationType(VisualizationType::MESH);
        object314_->SetSuspensionVisualizationType(VisualizationType::MESH);
        object314_->SetIdlerWheelVisualizationType(VisualizationType::MESH);
        object314_->SetRoadWheelVisualizationType(VisualizationType::MESH);
        object314_->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

        auto& vehicle = object314_->GetVehicle();
        vehicle.EnableRealtime(false);

        terrain_ = create_terrain(object314_->GetSystem());
        if constexpr (kEnableFixedObstacles) {
            add_fixed_obstacles(object314_->GetSystem());
        }

        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        object314_->GetSystem()->SetSolver(solver);
        object314_->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);

        set_chassis_velocity(requested_state);
        previous_wrapped_yaw_ = yaw_from_quaternion(object314_->GetChassisBody()->GetRot());
        unwrapped_yaw_ = requested_state.yaw;
        initialize_visualization(requested_state);
        return read_state();
    }

    PlantState step(double dt, double left_torque, double right_torque) {
        const int full_steps = static_cast<int>(std::floor(dt / kInternalStepSize));
        const double remainder = dt - full_steps * kInternalStepSize;

        for (int i = 0; i < full_steps; ++i) {
            advance(kInternalStepSize, left_torque, right_torque);
        }
        if (remainder > 0.0) {
            advance(remainder, left_torque, right_torque);
        }

        return read_state();
    }

    double get_time() const { return object314_->GetSystem()->GetChTime(); }

    PlantState read_state() {
        const auto chassis_body = object314_->GetChassisBody();
        const auto& frame = chassis_body->GetFrameRefToAbs();
        const ChVector3d local_velocity = frame.TransformDirectionParentToLocal(chassis_body->GetPosDt());
        const ChVector3d local_angular_velocity = chassis_body->GetAngVelLocal();
        const ChVector3d& position = chassis_body->GetPos();
        const double wrapped_yaw = yaw_from_quaternion(chassis_body->GetRot());
        unwrapped_yaw_ += wrap_angle(wrapped_yaw - previous_wrapped_yaw_);
        previous_wrapped_yaw_ = wrapped_yaw;

        PlantState state;
        state.x = position.x();
        state.y = position.y();
        state.yaw = unwrapped_yaw_;
        state.vx = local_velocity.x();
        state.vy = local_velocity.y();
        state.yaw_rate = local_angular_velocity.z();
        return state;
    }

  private:
    void initialize_visualization(const PlantState& requested_state) {
        if (!vis_) {
            vis_ = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_->SetWindowTitle("Object314 ROS Simulator");
            vis_->EnableStats(false);
            vis_->Initialize();
            vis_->AddLightDirectional(60, -90);
            vis_->AddSkyBox();
        }

        vis_->AttachVehicle(&object314_->GetVehicle());
        vis_->SetChaseCameraPosition(ChVector3d(requested_state.x, requested_state.y, kDefaultTopViewHeight),
                                     ChVector3d(requested_state.x, requested_state.y, 0.0));
        vis_->Advance(0.0);

        next_render_time_ = object314_->GetSystem()->GetChTime();
        render_visualization(object314_->GetSystem()->GetChTime(), DriverInputs{});
    }

    void render_visualization(double time, const DriverInputs& vehicle_inputs) {
        if (!vis_ || !vis_->Run()) {
            return;
        }

        vis_->Synchronize(time, vehicle_inputs);
        vis_->BeginScene();
        vis_->Render();
        vis_->EndScene();
    }

    void set_chassis_velocity(const PlantState& requested_state) {
        const auto chassis_body = object314_->GetChassisBody();
        const auto& frame = chassis_body->GetFrameRefToAbs();
        const ChVector3d absolute_velocity =
            frame.TransformDirectionLocalToParent(ChVector3d(requested_state.vx, requested_state.vy, 0.0));
        const ChVector3d absolute_angular_velocity =
            frame.TransformDirectionLocalToParent(ChVector3d(0.0, 0.0, requested_state.yaw_rate));
        chassis_body->SetPosDt(absolute_velocity);
        chassis_body->SetAngVelParent(absolute_angular_velocity);
    }

    void advance(double step, double left_torque, double right_torque) {
        DriverInputs vehicle_inputs = {};
        const double time = object314_->GetVehicle().GetChTime();
        terrain_->Synchronize(time);
        object314_->Synchronize(time, vehicle_inputs);
        object314_->ApplyTrackTorques(left_torque, right_torque);
        terrain_->Advance(step);
        object314_->Advance(step);
        if (vis_) {
            vis_->Advance(step);
        }

        const double new_time = object314_->GetVehicle().GetChTime();
        if (vis_ && new_time + 1e-12 >= next_render_time_) {
            render_visualization(new_time, vehicle_inputs);
            next_render_time_ += kRenderStepSize;
        }
    }

    std::unique_ptr<Object314> object314_;
    std::unique_ptr<ChTerrain> terrain_;
    std::shared_ptr<ChTrackedVehicleVisualSystemIrrlicht> vis_;
    double next_render_time_ = 0.0;
    double previous_wrapped_yaw_ = 0.0;
    double unwrapped_yaw_ = 0.0;
};

class Object314RosSimulatorNode : public rclcpp::Node {
  public:
    Object314RosSimulatorNode() : Node("object314_ros_sim", kDefaultRosNamespace) {
        reset_service_ = create_service<ResetSimulation>(
            "reset", [this](const std::shared_ptr<ResetSimulation::Request> request,
                            std::shared_ptr<ResetSimulation::Response> response) { handle_reset(request, response); });
        step_service_ = create_service<StepSimulation>(
            "step", [this](const std::shared_ptr<StepSimulation::Request> request,
                           std::shared_ptr<StepSimulation::Response> response) { handle_step(request, response); });
        RCLCPP_INFO(get_logger(), "Object314 ROS simulator ready: reset=%s step=%s", reset_service_->get_service_name(),
                    step_service_->get_service_name());
    }

  private:
    void handle_reset(const std::shared_ptr<ResetSimulation::Request> request,
                      std::shared_ptr<ResetSimulation::Response> response) {
        if (request->state.size() != 6) {
            response->success = false;
            response->message = "ResetSimulation.state must contain exactly 6 values: [X, Y, theta, Vx, Vy, omega].";
            response->time = 0.0;
            response->state = state_to_vector(plant_.read_state());
            return;
        }

        const PlantState state = plant_.reset(state_from_vector(request->state));
        response->success = true;
        response->message = "";
        response->time = plant_.get_time();
        response->state = state_to_vector(state);
    }

    void handle_step(const std::shared_ptr<StepSimulation::Request> request,
                     std::shared_ptr<StepSimulation::Response> response) {
        if (request->command_mode != StepSimulation::Request::COMMAND_MODE_TRACK_TORQUE) {
            response->success = false;
            response->message = "Object314 ROS simulator supports only COMMAND_MODE_TRACK_TORQUE.";
            response->time = plant_.get_time();
            response->state = state_to_vector(plant_.read_state());
            return;
        }
        if (request->command.size() != 2) {
            response->success = false;
            response->message = "StepSimulation.command must contain exactly 2 values: [tau_l, tau_r].";
            response->time = 0.0;
            response->state = state_to_vector(plant_.read_state());
            return;
        }
        if (!std::isfinite(request->dt) || request->dt <= 0.0 || request->dt > kMaxStepRequest) {
            response->success = false;
            response->message = "StepSimulation.dt must be finite and in the range (0, 1].";
            response->time = 0.0;
            response->state = state_to_vector(plant_.read_state());
            return;
        }

        const double left_torque = std::clamp(request->command[0], -kMaxTrackTorque, kMaxTrackTorque);
        const double right_torque = std::clamp(request->command[1], -kMaxTrackTorque, kMaxTrackTorque);
        const PlantState state = plant_.step(request->dt, left_torque, right_torque);
        response->success = true;
        response->message = "";
        response->time = plant_.get_time();
        response->state = state_to_vector(state);
    }

    Object314RosPlant plant_;
    rclcpp::Service<ResetSimulation>::SharedPtr reset_service_;
    rclcpp::Service<StepSimulation>::SharedPtr step_service_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Object314RosSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
