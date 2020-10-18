#include <iostream>
#include <vector>

#include "lcm/lcm-cpp.hpp"
#include "gflags/gflags.h"

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/traj_gen/admm_runner.h"
#include "drake/traj_gen/ddp_runner.h"
// #include "drake/traj_gen/ilqr_kkt/ddp_runner_contact.h"

#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/lcmt_motion_plan_query.hpp"
#include "drake/traj_gen/config.h"

DEFINE_bool(use_admm, false, "whether to use admm or ddp");

DEFINE_double(gripper_open_width, 100, "Width gripper opens to in mm");
DEFINE_double(gripper_close_width, 10, "Width gripper closes to in mm");
DEFINE_double(gripper_force, 50, "force for gripper");

DEFINE_string(
    KukaIiwaUrdf,
    "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf",
    "file name of iiwa7 urdf"
);

DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

DEFINE_string(plan_channel, "COMMITTED_ROBOT_PLAN", "Plan channels for plan");

using lcm::LCM;
using drake::manipulation::planner::ConstraintRelaxingIk;
using drake::traj_gen::kuka_iiwa_arm::ADMMRunner;
using drake::traj_gen::kuka_iiwa_arm::DDPRunner;

namespace drake {
namespace manipulation_tamp {


class TrajTestRunner {
public:
TrajTestRunner() {
    model_path_ = FindResourceOrThrow(FLAGS_KukaIiwaUrdf);
}

void Run(const lcmt_motion_plan_query* query) {
    current_query_ = query;

    ConstraintRelaxingIk::IkCartesianWaypoint wp;
    const Eigen::Vector3d xyz(
        query->desired_ee[0],
        query->desired_ee[1],
        query->desired_ee[2]
    );
    const math::RollPitchYaw<double> rpy(
        query->desired_ee[3],
        query->desired_ee[4],
        query->desired_ee[5]
    );

    wp.pose.set_translation(xyz);
    wp.pose.set_rotation(rpy);
    wp.constrain_orientation = true;

    Eigen::VectorXd iiwa_q(query->dim_q);
    for (int i = 0; i < query->dim_q; i++) {
        iiwa_q[i] = query->prev_q[i];
    }

    ConstraintRelaxingIk ik(
        model_path_,
        FLAGS_ee_name
    );

    VectorXd q_goal(7);
    std::vector<Eigen::VectorXd> ik_res;
    std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
    wp_vec.push_back(wp);
    if (ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)) {
        q_goal = ik_res.back();
    } else {
        std::cout<<"IK infeasible\n";
        return;
    }

    // q_goal << -1.43793, 0.71893, -0.313679, -1.17462, 0.213281, 1.27488, -0.169404;
    // q_goal << -1.00343, 0.997431, -0.744752, -1.41766, 0.73145, 1.0159, -0.339804;
    q_goal << 1.71627, -1.30988, -2.51169, -1.5873, -2.82295, 1.40921, 2.47325;
    lcmt_manipulator_traj traj;
    if (FLAGS_use_admm) {
        traj = GetADMMRes(iiwa_q, q_goal, query);
    } else {
        traj = GetDDPRes(iiwa_q, q_goal, query);
    }
    std::vector<double> widths;
    std::vector<double> forces;
    forces.assign(traj.n_time_steps, FLAGS_gripper_force);
    widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);

    traj.gripper_force = forces;
    traj.gripper_width = widths;

    std::cout<<"Goal: "<< q_goal<<"\n";
    std::cout<<"Press any key to continue...\n";
    while (std::getc(stdin)==EOF) {}

    lcm_.publish(FLAGS_plan_channel, &traj);
    std::cout<<"Trajectory Published\n";
}

private:
string model_path_;
LCM lcm_;
const lcmt_motion_plan_query* current_query_;

lcmt_manipulator_traj GetDDPRes(VectorXd q_init, VectorXd q_goal,
    const lcmt_motion_plan_query* query) {
    std::cout<<"IK Successful, sent to ddp\n";
    std::cout<<"ddp initial pos: " << q_init.transpose() << std::endl;
    std::cout<<"ddp goal pos: " << q_goal.transpose() << std::endl;

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";

    DDPRunner runner;
    return runner.RunDDP(qv_init, q_goal, query->time_horizon, query->time_step);
}

lcmt_manipulator_traj GetADMMRes(VectorXd q_init, VectorXd q_goal,
    const lcmt_motion_plan_query* query) {
    std::cout<<"IK Successful, sent to admm\n";
    std::cout<<"admm initial pos: " << q_init.transpose() << std::endl;
    std::cout<<"admm goal pos: " << q_goal.transpose() << std::endl;

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";
    ADMMRunner runner;
    return runner.RunADMM(qv_init, q_goal, query->time_horizon, query->time_step, query->name);
}

};

} // namespace tamp_conveyor_belt
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::manipulation_tamp::TrajTestRunner runner;
    drake::lcmt_motion_plan_query query;
    query.dim_q = 7;
    query.name = "push";
    query.level = 0;
    query.time_horizon = 1.0;
    query.time_step = 0.005;
    query.wait_time = 0;

    // double prev_q[] = {0, 0, 0, 0, 0, 0, 0};
    // double prev_q[] = {-0.133372, 0.251457, -0.0461879, -1.21048, 0.0324702, 0.928553, -0.190112};
    // double prev_q[] = {1.84849, 1.30959, -0.0757701, -1.37273, -1.29295, 1.59139, 2.68207}; //for pushing
    double prev_q[] = {-0.099479, 1.80031, -1.69171, -1.72316, 2.56677, 0.312838, -0.777406};
    // double prev_q[] = {-0.935018, 0.786996, -0.90662, -1.31144, 0.614953, 1.32472, -0.268566};
    // double prev_q[] = {-0.9498766005895738, -1.4303909653637479, 2.0864686773500476, -1.4801119967595946, 0.11195986419142938, 0.889741592707635, -0.003942442475240289};
    query.prev_q = std::vector<double>(prev_q, prev_q+sizeof(prev_q)/sizeof(double));

    // query.desired_ee[0] = 0.4;
    // query.desired_ee[1] = 0.05;
    // query.desired_ee[2] = 0.35;
    // query.desired_ee[3] = 0;
    // query.desired_ee[4] = 1.57;
    // query.desired_ee[5] = -1.57;
    
    // for pushing
    query.desired_ee[0] = 0.5;
    query.desired_ee[1] = 0.55;
    query.desired_ee[2] = 0.0816099;
    query.desired_ee[3] = 1.42092e-12;
    query.desired_ee[4] = 0.0292037;
    query.desired_ee[5] = 4.26875e-12;
    // query.desired_ee[0] = 0.5656;
    // query.desired_ee[1] = 0.0;
    // query.desired_ee[2] = 0.25;
    // query.desired_ee[3] = 0;
    // query.desired_ee[4] = 0;
    // query.desired_ee[5] = -0.7854;

    runner.Run(&query);
}