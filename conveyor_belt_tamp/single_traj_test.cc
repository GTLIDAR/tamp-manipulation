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

#include "drake/lcmt_ddp_traj.hpp"
#include "drake/lcmt_motion_plan_query.hpp"

DEFINE_bool(use_admm, true, "whether to use admm or ddp");

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
namespace conveyor_belt_tamp {


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

    lcmt_ddp_traj traj;
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

    std::cout<<"Press any key to continue...\n";
    while (std::getc(stdin)==EOF) {}

    lcm_.publish(FLAGS_plan_channel, &traj);
    std::cout<<"Trajectory Published\n";
}

private:
string model_path_;
LCM lcm_;
const lcmt_motion_plan_query* current_query_;

lcmt_ddp_traj GetDDPRes(VectorXd q_init, VectorXd q_goal, 
    const lcmt_motion_plan_query* query) {
    std::cout<<"IK Successful, sent to ddp\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";

    DDPRunner runner;
    return runner.RunUDP(qv_init, q_goal, query);
}

lcmt_ddp_traj GetADMMRes(VectorXd q_init, VectorXd q_goal, 
    const lcmt_motion_plan_query* query) {
    std::cout<<"IK Successful, sent to admm\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";
    ADMMRunner runner;
    return runner.RunADMM(qv_init, q_goal, query);
}

};

} // namespace tamp_conveyor_belt
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::conveyor_belt_tamp::TrajTestRunner runner;
    drake::lcmt_motion_plan_query query;
    query.dim_q = 7;
    query.name = "Test";
    query.level = 0;
    query.time_horizon = 2;
    query.time_step = 0.005;
    query.wait_time = 0;

    double prev_q[] = {0, 0, 0, 0, 0, 0, 0};
    query.prev_q = std::vector<double>(prev_q, prev_q+sizeof(prev_q)/sizeof(double));

    query.desired_ee[0] = 0.4;
    query.desired_ee[1] = 0.1;
    query.desired_ee[2] = 0.3;
    query.desired_ee[3] = 0;
    query.desired_ee[4] = 0;
    query.desired_ee[5] = -1.57;
    
    runner.Run(&query);
}