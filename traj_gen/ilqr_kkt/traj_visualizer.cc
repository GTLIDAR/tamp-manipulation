#include <limits>
#include <string>
#include <fstream>

#include <gflags/gflags.h>
#include <jsoncpp/json/json.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/find_resource.h"
#include "drake/traj_gen/ilqr_kkt/robot_time_sender.h"
#include "drake/traj_gen/ilqr_kkt/object/object_status_sender.h"
#include "drake/traj_gen/ilqr_kkt/object/object_status_receiver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_generic_string_msg.hpp"
#include "drake/lcmt_combined_object_state.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_object_status.hpp"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using lcm::LCM;
using drake::examples::kuka_iiwa_arm::IiwaStatusReceiver;
using drake::examples::kuka_iiwa_arm::IiwaStatusSender;
using drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using drake::manipulation::kuka_iiwa::kIiwaArmNumJoints;

using drake::manipulation::schunk_wsg::SchunkWsgStatusSender;
using drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver;
using drake::manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod;
using drake::manipulation::schunk_wsg::kSchunkWsgNumPositions;

using drake::traj_gen::kuka_iiwa_arm::ObjectStatusReceiver;
using drake::traj_gen::kuka_iiwa_arm::ObjectStatusSender;
using drake::traj_gen::kuka_iiwa_arm::kObjectLcmStatusPeriod;

using drake::traj_gen::kuka_iiwa_arm::RobotTimeSender;

using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::ModelInstanceIndex;
using drake::systems::Simulator;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

void DoMain(){
    // Build a multibody plant.
    systems::DiagramBuilder<double> builder;
    MultibodyPlant<double> plant_(1e-5);
    plant_.set_name("plant");

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    Parser parser(&plant_, &scene_graph);

    std::string kIiwaUrdf = 
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string schunkPath = 
        FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf");
    std::string connectorPath = 
        FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");
    // const std::string box_sdf_path0 = "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf";
    // const std::string box_sdf_path0 = "drake/manipulation_tamp/models/boxes/redblock.urdf";
    const std::string box_sdf_path0 = "drake/manipulation_tamp/models/sphere.sdf";
    
    // For pushing we use a larger box
    // const std::string box_sdf_path0 = "drake/manipulation_tamp/models/boxes/large_red_box.urdf";

    const ModelInstanceIndex iiwa_model = 
        parser.AddModelFromFile(kIiwaUrdf, "iiwa");
    const auto& iiwa_base_frame = plant_.GetFrameByName("iiwa_link_0", iiwa_model);
    RigidTransformd X_WI(Eigen::Vector3d(0, 0, 0));
    plant_.WeldFrames(plant_.world_frame(), iiwa_base_frame, X_WI);

    const ModelInstanceIndex conn_model = 
        parser.AddModelFromFile(connectorPath, "connector");
    const auto& iiwa_ee_frame = plant_.GetFrameByName("iiwa_frame_ee", iiwa_model);
    const auto& conn_frame = plant_.GetFrameByName("connector_link", conn_model);
    RigidTransformd X_EC(Eigen::Vector3d(0, 0, 0));
    plant_.WeldFrames(iiwa_ee_frame, conn_frame, X_EC);

    const ModelInstanceIndex wsg_model = 
        parser.AddModelFromFile(schunkPath, "wsg");
    const auto& wsg_frame = plant_.GetFrameByName("body", wsg_model);
    RigidTransformd X_EG(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.053));
    plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

    // const ModelInstanceIndex object_model =
    parser.AddModelFromFile(FindResourceOrThrow(box_sdf_path0), "object");

    plant_.Finalize();

    unsigned int num_robots = 3;
    std::vector<int> all_joints;
    all_joints.resize(num_robots,0);
    all_joints[0] = 7;
    all_joints[1] = kIiwaArmNumJoints;
    all_joints[2] = kSchunkWsgNumPositions;
    // ------------------------- SYSTEM CREATION ---------------------------------
    // Create LCM
    auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
    // iiwa lcm
    auto iiwa_status_sub = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS", lcm));
    iiwa_status_sub->set_name("iiwa_state_subscriber");
    auto iiwa_status_rec = builder.AddSystem<IiwaStatusReceiver>(kIiwaArmNumJoints);
    iiwa_status_rec->set_name("iiwa_status_receiver");

    auto iiwa_time_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
            "IIWA_TIME", lcm, kIiwaLcmStatusPeriod /* publish period */));
    iiwa_time_pub->set_name("iiwa_time_publisher");
    auto iiwa_time_sender = builder.AddSystem<RobotTimeSender>();
    iiwa_time_sender->set_name("iiwa_time_sender");

    // wsg lcm
    auto wsg_status_sub = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
      "WSG_STATUS", lcm));
    wsg_status_sub->set_name("wsg_state_subscriber");
    auto wsg_status_rec = builder.AddSystem<SchunkWsgStatusReceiver>();
    wsg_status_rec->set_name("wsg_status_receiver");
    
    // object lcm
    auto object_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_object_status>(
          "OBJECT_STATUS", lcm));
    object_state_subscriber->set_name("object_state_subscriber");
    auto object_state_receiver =
        builder.AddSystem<ObjectStatusReceiver>(7);
    object_state_receiver->set_name("object_state_receiver");

    // Plug in all plant state quantities
    std::vector<int> mux_sizes;
    for(unsigned int i = 0; i < num_robots; i++)
    {
        mux_sizes.push_back(all_joints[i]);
    }
    DRAKE_DEMAND(mux_sizes.size() == num_robots);
    auto plant_states_mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);    

    // Robot states to geometry visualization system
    auto robot_to_pose =
            builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant_);
    // iiwa
    builder.Connect(iiwa_status_sub->get_output_port(),
                    iiwa_status_rec->get_input_port());
    builder.Connect(iiwa_status_rec->get_position_measured_output_port(),
                    plant_states_mux->get_input_port(1));
    builder.Connect(iiwa_time_sender->get_output_port(),
                    iiwa_time_pub->get_input_port());

    // wsg
    builder.Connect(wsg_status_sub->get_output_port(),
                    wsg_status_rec->get_status_input_port());
    builder.Connect(wsg_status_rec->get_state_output_port(),
                    plant_states_mux->get_input_port(2));

    // obj
    builder.Connect(object_state_subscriber->get_output_port(),
                    object_state_receiver->get_input_port());
    builder.Connect(object_state_receiver->get_position_measured_output_port(),
                    plant_states_mux->get_input_port(0));

    // Visualization connections
    builder.Connect(plant_states_mux->get_output_port(0),
                    robot_to_pose->get_input_port());
    builder.Connect(robot_to_pose->get_output_port(), 
                    scene_graph.get_source_pose_port(plant_.get_source_id().value()));
    
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();

    // TODO: Set initial states for object and kuka (just for better visualization)
    // Eigen::VectorXd init_multi_object_state(7);
    // for(int i = 0; i < 7; i++)
    // {
    //     init_multi_object_state[i] = root["multi_object_position_1"][temp][i].asDouble();
    // }
    // multi_object_state_receiver->SetInitialPosition(init_multi_object_state);

    // Set up simulator.
    auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
    systems::Context<double>& root_context = simulator->get_mutable_context();

    // Context: 3 abs, 0 cont, 0 dis
    drake::log()->info("context absStates: {}",root_context.num_abstract_states());
    drake::log()->info("context contStates: {}",root_context.num_continuous_states());
    drake::log()->info("context disStates: {}",root_context.num_discrete_state_groups());

    simulator->set_target_realtime_rate(1.0);
    simulator->Initialize();
    simulator->AdvanceTo(std::numeric_limits<double>::infinity());
}

} // namespace kuka_iiwa_arm
} // namespace traj_gen
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
        "Visualizer for robot collaboration simulation");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::traj_gen::kuka_iiwa_arm::DoMain();
    return 0;
}