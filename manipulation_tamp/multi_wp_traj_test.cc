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

#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/lcmt_multi_wp_manip_query.hpp"

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

void Run(const lcmt_multi_wp_manip_query* query) {
    // initialize
    VectorXd iiwa_q = VectorXd::Zero(query->dim_q);
    for (int i = 0; i < query->dim_q; i++) {
        iiwa_q[i] = query->prev_q[i];
    }

    ConstraintRelaxingIk ik(
        model_path_,
        FLAGS_ee_name
    );

    std::vector<VectorXd> q_sol;
    std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
    
    for (int i = 0; i < query->n_wp; i++) {
        ConstraintRelaxingIk::IkCartesianWaypoint wp;
        const Eigen::Vector3d xyz(
            query->desired_ee[i][0],
            query->desired_ee[i][1],
            query->desired_ee[i][2]
        );
        const math::RollPitchYaw<double> rpy(
            query->desired_ee[i][3],
            query->desired_ee[i][4],
            query->desired_ee[i][5]
        );

        wp.pose.set_translation(xyz);
        wp.pose.set_rotation(rpy);
        wp.constrain_orientation = true;

        wp_vec.push_back(wp);
        std::cout<<"WP added "<<xyz<<"\n";
    }

    bool ik_feasible = ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &q_sol);

    if (!ik_feasible) {
        std::cout<<"IK infeasuble\n";
        return;
    }

    lcmt_manipulator_traj total_traj;
    total_traj.dim_states = query->dim_q;
    total_traj.dim_torques = 0;
    total_traj.n_time_steps = 0;

    lcmt_manipulator_traj traj;
    for (size_t i = 0; i < q_sol.size()-1; i++) {
        if (query->option.compare("ddp")==0) {
            traj = 
                GetDDPRes(q_sol[i], q_sol[i+1], query->time_horizon[i], query->time_step);
        } else if (query->option.compare("admm")==0) {
            traj = 
                GetADMMRes(q_sol[i], q_sol[i+1], query->time_horizon[i], query->time_step, query->name);
        } else {
            std::cout<<"Invalid Traj Op Option. Only support ddp or admm!\n";
        }

        std::vector<double> widths;
        std::vector<double> forces;
        forces.assign(traj.n_time_steps, FLAGS_gripper_force);
        widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);
        traj.gripper_force = forces;
        traj.gripper_width = widths;

        AppendTrajectory(total_traj, traj);
        std::cout<<"Traj Appended\n";
        std::cout << "Traj Length: " << total_traj.n_time_steps << "\n";
    }

    std::cout<<"Press any key to continue...\n";
    while (std::getc(stdin)==EOF) {}

    lcm_.publish(FLAGS_plan_channel, &total_traj);
    std::cout<<"Trajectory Published\n";
}

private:
string model_path_;
LCM lcm_;

lcmt_manipulator_traj GetDDPRes(VectorXd q_init, VectorXd q_goal,
    double time_horizon, double time_step) {
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
    return runner.RunDDP(qv_init, qv_goal, time_horizon, time_step);
}

lcmt_manipulator_traj GetADMMRes(VectorXd q_init, VectorXd q_goal,
    double time_horizon, double time_step, string action_name) {
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
    return runner.RunADMM(qv_init, qv_goal, time_horizon, time_step, action_name);
}

void AppendTrajectory(lcmt_manipulator_traj &dest, lcmt_manipulator_traj &src) {
    DRAKE_DEMAND(dest.dim_states==src.dim_states);
    DRAKE_DEMAND(dest.dim_torques==src.dim_torques);

    dest.n_time_steps += src.n_time_steps;
    dest.cost += src.cost;
    std::vector<double> times_sec = src.times_sec;
    if (dest.times_sec.size()) {
        for (size_t i = 0; i < times_sec.size(); i++) {
            times_sec[i] += dest.times_sec.back();
        }
    }

    dest.times_sec.insert(
        dest.times_sec.end(), times_sec.begin(), times_sec.end());
    dest.states.insert(
        dest.states.end(), src.states.begin(), src.states.end()); 
    dest.torques.insert(
        dest.torques.end(), src.torques.begin(), src.torques.end());
    dest.gripper_width.insert(
        dest.gripper_width.end(), src.gripper_width.begin(), src.gripper_width.end());
    dest.gripper_force.insert(
        dest.gripper_force.end(), src.gripper_force.begin(), src.gripper_force.end());
}

};

} // namespace tamp_conveyor_belt
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::manipulation_tamp::TrajTestRunner runner;
    drake::lcmt_multi_wp_manip_query query;
    
    query.dim_q = 7;
    query.n_wp = 3;
    query.name = "Test";
    if (FLAGS_use_admm) {
        query.option = "admm";
    } else {
        query.option = "ddp";
    }

    query.time_step = 0.005;

    double prev_q[] = {0, 0, 0, 0, 0, 0, 0};
    query.prev_q = std::vector<double>(prev_q, prev_q+sizeof(prev_q)/sizeof(double));
    double time_horizon[] = {3, 3, 3};
    query.time_horizon = std::vector<double>(time_horizon, time_horizon+sizeof(time_horizon)/sizeof(double));

    query.gripper_force = FLAGS_gripper_force;
    query.prev_gripper_width = FLAGS_gripper_close_width;

    std::vector<double> wp0 = {0.4, 0.05, 0.35, 0, 1.57, -1.57};
    std::vector<double> wp1 = {0.4, 0.05, 0.2, 0, 1.57, -1.57};
    std::vector<double> wp2 = {0.4, 0.05, 0.45, 0, 0, -1.57};

    query.desired_ee.push_back(wp0);
    query.desired_ee.push_back(wp1);
    query.desired_ee.push_back(wp2);

    runner.Run(&query);
}
