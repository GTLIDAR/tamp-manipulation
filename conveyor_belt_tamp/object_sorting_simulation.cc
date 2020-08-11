#include <limits>
#include <string>
#include <fstream>

#include <gflags/gflags.h>
#include <jsoncpp/json/json.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/conveyor_belt_tamp/manipulation_station.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
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

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_generic_string_msg.hpp"
#include "drake/lcmt_combined_object_state.hpp"

using lcm::LCM;
namespace drake {
namespace conveyor_belt_tamp {
namespace manipulation_station {
DEFINE_string(geo_setup_file, "drake/conveyor_belt_tamp/setup/object_sorting/geo_setup.json",
    "file for geometry setup");
DEFINE_string(sim_setup_file, "drake/conveyor_belt_tamp/setup/sim_setup.json",
    "file for simulation setup");

using examples::kuka_iiwa_arm::IiwaCommandReceiver;
using examples::kuka_iiwa_arm::IiwaStatusSender;

int do_main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Json::Value geo_setup;
    std::ifstream geo_fstream(FindResourceOrThrow(FLAGS_geo_setup_file));
    geo_fstream >> geo_setup;

    Json::Value sim_setup;
    std::ifstream sim_fstream(FindResourceOrThrow(FLAGS_sim_setup_file));
    sim_fstream >> sim_setup;

    auto object_init_pos = geo_setup["object_init_pos"];

    systems::DiagramBuilder<double> builder;
    auto station = builder.AddSystem<ManipulationStation>(
        sim_setup["discrete_update_period"].asDouble());

    station->SetupObjectSortingStation();
    if (sim_setup["enable_objects"].asBool()) {
    // setup objects
    auto kConveyorBeltTopZInWorld = geo_setup["kConveyorBeltTopZInWorld"].asDouble();
    auto xAdditionalOffset = geo_setup["xAdditionalOffset"].asDouble();
    auto yAdditionalOffset = geo_setup["yAdditionalOffset"].asDouble();

    // box_0 first red box
    {
    const std::string box_sdf_path0 = "drake/conveyor_belt_tamp/models/boxes/redblock.urdf";
    
    auto rpy = math::RollPitchYawd(Eigen::Vector3d(
        object_init_pos["box_0"][3].asDouble(),
        object_init_pos["box_0"][4].asDouble(),
        object_init_pos["box_0"][5].asDouble()
    ));

    auto xyz = Eigen::Vector3d(
        object_init_pos["box_0"][0].asDouble() + xAdditionalOffset,
        object_init_pos["box_0"][1].asDouble() + yAdditionalOffset,
        object_init_pos["box_0"][2].asDouble() + kConveyorBeltTopZInWorld
    );

    math::RigidTransform<double> X_WO(
        math::RotationMatrix<double>(rpy),
        xyz
    );
    
    station->AddManipulandFromFile(box_sdf_path0, X_WO);
    }
    // box_1 outside black box
    {
    const std::string box_sdf_path1 = "drake/conveyor_belt_tamp/models/boxes/black_box.urdf";
    
    auto rpy = math::RollPitchYawd(Eigen::Vector3d(
        object_init_pos["box_1"][3].asDouble(),
        object_init_pos["box_1"][4].asDouble(),
        object_init_pos["box_1"][5].asDouble()
    ));

    auto xyz = Eigen::Vector3d(
        object_init_pos["box_1"][0].asDouble() + xAdditionalOffset,
        object_init_pos["box_1"][1].asDouble() + yAdditionalOffset,
        object_init_pos["box_1"][2].asDouble() + kConveyorBeltTopZInWorld
    );

    math::RigidTransform<double> X_W1(
        math::RotationMatrix<double>(rpy),
        xyz
    );
    
    station->AddManipulandFromFile(box_sdf_path1, X_W1);
    }

    // box_2 inside black box
    {
    const std::string black_box_urdf_path2 = "drake/conveyor_belt_tamp/models/boxes/black_box4.urdf";
    
    auto rpy = math::RollPitchYawd(Eigen::Vector3d(
        object_init_pos["box_2"][3].asDouble(),
        object_init_pos["box_2"][4].asDouble(),
        object_init_pos["box_2"][5].asDouble()
    ));

    auto xyz = Eigen::Vector3d(
        object_init_pos["box_2"][0].asDouble() + xAdditionalOffset,
        object_init_pos["box_2"][1].asDouble() + yAdditionalOffset,
        object_init_pos["box_2"][2].asDouble() + kConveyorBeltTopZInWorld
    );

    math::RigidTransform<double> X_W2(
        math::RotationMatrix<double>(rpy),
        xyz
    );
    
    station->AddManipulandFromFile(black_box_urdf_path2, X_W2);
    }

    // box_3 red box behind first big box
    {
    const std::string box_urdf_path = "drake/conveyor_belt_tamp/models/boxes/redblock2.urdf";
    
    auto rpy = math::RollPitchYawd(Eigen::Vector3d(
        object_init_pos["box_3"][3].asDouble(),
        object_init_pos["box_3"][4].asDouble(),
        object_init_pos["box_3"][5].asDouble()
    ));

    auto xyz = Eigen::Vector3d(
        object_init_pos["box_3"][0].asDouble() + xAdditionalOffset,
        object_init_pos["box_3"][1].asDouble() + yAdditionalOffset,
        object_init_pos["box_3"][2].asDouble() + kConveyorBeltTopZInWorld
    );

    math::RigidTransform<double> X_W3(
        math::RotationMatrix<double>(rpy),
        xyz
    );
    
    station->AddManipulandFromFile(box_urdf_path, X_W3);
    }

    // box_4
    {
    const std::string box_urdf_path = "drake/conveyor_belt_tamp/models/boxes/black_box3.urdf";
    
    auto rpy = math::RollPitchYawd(Eigen::Vector3d(
        object_init_pos["box_4"][3].asDouble(),
        object_init_pos["box_4"][4].asDouble(),
        object_init_pos["box_4"][5].asDouble()
    ));

    auto xyz = Eigen::Vector3d(
        object_init_pos["box_4"][0].asDouble() + xAdditionalOffset,
        object_init_pos["box_4"][1].asDouble() + yAdditionalOffset,
        object_init_pos["box_4"][2].asDouble() + kConveyorBeltTopZInWorld
    );

    math::RigidTransform<double> X_W3(
        math::RotationMatrix<double>(rpy),
        xyz
    );
    
    station->AddManipulandFromFile(box_urdf_path, X_W3);
    }

    }

    station->Finalize();
    geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
        station->GetOutputPort("pose_bundle"));

    auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

    // add lcm systems for iiwa
    auto iiwa_command_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
            "IIWA_COMMAND", lcm
        )
    );
    auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>();

    auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();
    auto iiwa_status_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
            "IIWA_STATUS", lcm, 0.005 // publish period
        )
    );
    // connect iiwa lcm systems
    builder.Connect(iiwa_command_subscriber->get_output_port(),
                    iiwa_command_receiver->get_message_input_port());

    builder.Connect(iiwa_command_receiver->get_commanded_position_output_port(),
                    station->GetInputPort("iiwa_position"));
    builder.Connect(iiwa_command_receiver->get_commanded_torque_output_port(),
                    station->GetInputPort("iiwa_feedforward_torque"));
    builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                    iiwa_status_sender->get_position_commanded_input_port());
    builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                    iiwa_status_sender->get_position_measured_input_port());
    builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                    iiwa_status_sender->get_velocity_estimated_input_port());
    builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                    iiwa_status_sender->get_torque_commanded_input_port());
    builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                    iiwa_status_sender->get_torque_measured_input_port());
    builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                    iiwa_status_sender->get_torque_external_input_port());

    builder.Connect(iiwa_status_sender->get_output_port(),
                    iiwa_status_publisher->get_input_port());

    // add wsg lcm systems
    auto wsg_command_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_command>(
            "SCHUNK_WSG_COMMAND", lcm
        )
    );
    auto wsg_command_receiver = builder.AddSystem<
        manipulation::schunk_wsg::SchunkWsgCommandReceiver>();
    auto wsg_status_sender = builder.AddSystem<
        manipulation::schunk_wsg::SchunkWsgStatusSender>();
    auto wsg_status_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_status>(
            "SCHUNK_WSG_STATUS", lcm, 0.05 // publish period
        )
    );

    // connect wsg lcm systems
    builder.Connect(wsg_command_subscriber->get_output_port(),
                    wsg_command_receiver->GetInputPort("command_message"));
    builder.Connect(wsg_command_receiver->get_position_output_port(),
                    station->GetInputPort("wsg_position"));
    builder.Connect(wsg_command_receiver->get_force_limit_output_port(),
                    station->GetInputPort("wsg_force_limit"));
    builder.Connect(station->GetOutputPort("wsg_state_measured"),
                    wsg_status_sender->get_state_input_port());
    builder.Connect(station->GetOutputPort("wsg_force_measured"),
                    wsg_status_sender->get_force_input_port());
    builder.Connect(wsg_status_sender->get_output_port(0),
                    wsg_status_publisher->get_input_port());

    // add object lcm systems
    auto object_state_pub =
        builder.AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_combined_object_state>(
            "OBJECT_STATE", lcm, 0.05 // publish period
        ));

    // connect object lcm systems
    builder.Connect(station->GetOutputPort("object_states"),
                    object_state_pub->get_input_port());

    auto diagram = builder.Build();

    // setup simulator
    systems::Simulator<double> simulator(*diagram);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(sim_setup["target_realtime_rate"].asDouble());
    simulator.Initialize();

    LCM lcm_;
    lcmt_generic_string_msg msg;
    msg.msg = "start";
    lcm_.publish("START_PLAN", &msg);

    if (sim_setup.isMember("duration")) {
        simulator.AdvanceTo(sim_setup["duration"].asDouble());
    } else {
        simulator.AdvanceTo(std::numeric_limits<double>::infinity());
    }
    
    return 0;
}

} // namespace manipulation_station
} // namespace examples
} // namespace drake

int main(int argc, char* argv[]) {
    return drake::conveyor_belt_tamp::manipulation_station::do_main(argc, argv);
}
