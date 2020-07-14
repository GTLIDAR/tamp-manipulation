#include <limits>
#include <gflags/gflags.h>
#include <string>

#include "lcm/lcm-cpp.hpp"

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
DEFINE_double(conveyor_velocity, 0.1, "Velocity of conveyor belt");
DEFINE_double(
    target_real_time,
    1,
    "Playback Speed, See documentation for Simulator::set_target_realtime_rate()"
);
DEFINE_double(
    duration, std::numeric_limits<double>::infinity(), "Simulation duration");
DEFINE_string(setup, "conveyor_belt", "Manipulation station type to simulate");
DEFINE_double(table_width, 0.7112, "Width of table supporting kuka arm");
DEFINE_double(belt_width, 0.4, "Width of conveyor belt");
DEFINE_double(dt, 1e-3, "Integration step size");

using examples::kuka_iiwa_arm::IiwaCommandReceiver;
using examples::kuka_iiwa_arm::IiwaStatusSender;

int do_main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    systems::DiagramBuilder<double> builder;

    auto station = builder.AddSystem<ManipulationStation>();

    station->SetupConveyorBeltStation();
    // setup objects
    const double kConveyorBeltTopZInWorld = 0.736 + 0.02 / 2;
    // const double xAdditionalOffset = 0.06;
    const double yAdditionalOffset = -0.02;

    // box_0 first red box
    const std::string box_sdf_path = "drake/conveyor_belt_tamp/models/boxes/redblock.urdf";
    math::RigidTransform<double> X_WO1(
        math::RotationMatrix<double>::Identity(),
        Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.03,
                        0 + yAdditionalOffset,//-1.8,
                        kConveyorBeltTopZInWorld+0.1)
    );
    station->AddManipulandFromFile(box_sdf_path, X_WO1);

    // box_1 outside black box
    const std::string box_sdf_path3 = "drake/conveyor_belt_tamp/models/boxes/black_box.urdf";
    math::RigidTransform<double> X_WO3(
        math::RotationMatrix<double>::Identity(),
        Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.1,
                        -0.4 + yAdditionalOffset,//-1.8,
                        kConveyorBeltTopZInWorld+0.1)
    );
    station->AddManipulandFromFile(box_sdf_path3, X_WO3);

    // box_2 inside black box
    const std::string black_box_urdf_path4 = "drake/conveyor_belt_tamp/models/boxes/black_box4.urdf";
    math::RigidTransform<double> X_WO7(
        math::RotationMatrix<double>::Identity(),
        Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2-0.1,
                        -0.4 + yAdditionalOffset,
                        kConveyorBeltTopZInWorld+0.1)
    );
    station->AddManipulandFromFile(black_box_urdf_path4, X_WO7);

    // // box_3 first large box
    // const std::string large_box_sdf_path01 = "drake/conveyor_belt_tamp/models/boxes/large_red_box2.urdf";
    // math::RigidTransform<double> X_WOO1(
    //     math::RotationMatrix<double>::MakeYRotation(-M_PI_2), //math::RotationMatrix<double>::Identity()
    //     Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.01, //0.01
    //                     -1.0 + yAdditionalOffset, //-1
    //                     kConveyorBeltTopZInWorld+0.1)
    // );
    // station->AddManipulandFromFile(large_box_sdf_path01, X_WOO1);

    // // box_4 red box behind first big box
    // const std::string box_sdf_path2 = "drake/conveyor_belt_tamp/models/boxes/redblock2.urdf";
    // math::RigidTransform<double> X_WO2(
    //     math::RotationMatrix<double>::Identity(),
    //     Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.11,
    //                     -1.35 + yAdditionalOffset,//-1.8,
    //                     kConveyorBeltTopZInWorld+0.1)
    // );
    // station->AddManipulandFromFile(box_sdf_path2, X_WO2);

    // // box_5 black box before second large box
    // const std::string black_box_urdf_path3 = "drake/conveyor_belt_tamp/models/boxes/black_box3.urdf";
    // math::RigidTransform<double> X_WO6(
    //     math::RotationMatrix<double>::Identity(),
    //     Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.012,
    //                     -1.75 + yAdditionalOffset,
    //                     kConveyorBeltTopZInWorld+0.1)
    // );
    // station->AddManipulandFromFile(black_box_urdf_path3, X_WO6);

    // // box_6 second large box
    // const std::string large_box_sdf_path = "drake/conveyor_belt_tamp/models/boxes/large_red_box.urdf";
    // math::RigidTransform<double> X_WO4(
    //     math::RotationMatrix<double>::MakeYRotation(-M_PI_2), //math::RotationMatrix<double>::Identity()
    //     Eigen::Vector3d((FLAGS_belt_width+FLAGS_table_width)/2+0.01, //0.01
    //                     -2.3 + yAdditionalOffset, //-1
    //                     kConveyorBeltTopZInWorld+0.1)
    // );
    // station->AddManipulandFromFile(large_box_sdf_path, X_WO4);

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
    // simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram,
    // FLAGS_dt, &simulator.get_mutable_context());
    simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(FLAGS_dt);
    auto& context = simulator.get_mutable_context();
    auto& state = context.get_mutable_state();
    // auto& station_context = diagram->GetMutableSubsystemContext(*station, &context);

    auto& plant = station->get_multibody_plant();
    plant.SetVelocities(
        context,
        &state,
        station->GetConveyorBeltId(),
        drake::Vector1d(FLAGS_conveyor_velocity)
    );

    // Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);
    // iiwa_command_receiver->set_initial_position(
    //     &diagram->GetMutableSubsystemContext(*iiwa_command_receiver, &context), q0
    // );

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_real_time);
    simulator.Initialize();

    LCM lcm_;
    lcmt_generic_string_msg msg;
    msg.msg = "start";
    lcm_.publish("START_PLAN", &msg);

    simulator.AdvanceTo(FLAGS_duration);

    return 0;
}

} // namespace manipulation_station
} // namespace examples
} // namespace drake

int main(int argc, char* argv[]) {
    return drake::conveyor_belt_tamp::manipulation_station::do_main(argc, argv);
}
