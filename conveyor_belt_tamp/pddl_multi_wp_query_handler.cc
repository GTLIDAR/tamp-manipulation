#include <iostream>
#include <vector>
#include <string>

#include "gflags/gflags.h"
#include "lcm-cpp.hpp"

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/lcmt_multi_wp_manip_query.hpp"
#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/traj_gen/ddp_runner.h"
#include "drake/traj_gen/admm_runner.h"

DEFINE_double(gripper_open_width, 100, "Width gripper opens to in mm");
DEFINE_double(gripper_close_width, 10, "Width gripper closes to in mm");

DEFINE_string(
    KukaIiwaUrdf,
    "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf",
    "file name of iiwa7 urdf"
);
DEFINE_string(
    query_channel,
    "TREE_SEARCH_QUERY",
    "channel for tree search to send its path plan query"
);
DEFINE_string(
    result_channel,
    "TREE_SEARCH_QUERY_RESULTS",
    "channel for low level to send back query results"
);
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

using drake::manipulation::planner::ConstraintRelaxingIk;
using drake::traj_gen::kuka_iiwa_arm::ADMMRunner;
using drake::traj_gen::kuka_iiwa_arm::DDPRunner;
namespace real_lcm = lcm;

namespace drake {
namespace conveyor_belt_tamp {

class PDDLQueryHandler {

public:
PDDLQueryHandler() {
    // Q_POST_THROW_ << -3.40486e-12, 0.743339, 8.16463e-12,  -0.5, 8.20197e-12,
    //     -0.6 ,-1.01735e-11;
    model_path_ = FindResourceOrThrow(FLAGS_KukaIiwaUrdf);
    lcm_.subscribe(FLAGS_query_channel, &PDDLQueryHandler::HandleQuery, this);
}

void Run() {
    while (lcm_.handle()>=0);
}

private:
std::string model_path_;
real_lcm::LCM lcm_;
lcmt_multi_wp_manip_query current_query_;

void HandleQuery(
    const real_lcm::ReceiveBuffer*,
    const std::string&,
    const lcmt_multi_wp_manip_query* query
) {
    current_query_ = *query;
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
        wp.constrain_orientation = query->constrain_orientation[i];

        wp_vec.push_back(wp);
        std::cout<<"WP added "<<xyz<<"\n";
    }

    bool ik_feasible = ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &q_sol);

    lcmt_manipulator_traj traj;
    if (!ik_feasible) {
        std::cout<<"IK infeasuble\n";
        traj = GetInfCost();
        lcm_.publish(FLAGS_result_channel, &traj);
        return;
    }

    std::cout<<"\n";
    lcmt_manipulator_traj total_traj;
    total_traj.dim_states = query->dim_q;
    total_traj.dim_torques = 0;
    total_traj.n_time_steps = 0;

    for (size_t i = 0; i < q_sol.size()-1; i++) {
        std::cout<<"IK Results"<<"\n";
        std::cout<<"q_init ";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i][j]<<" ";
        }
        std::cout<<"\nq_goal";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i+1][j]<<" ";
        }
        std::cout<<"\n";

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
        forces.assign(traj.n_time_steps, query->gripper_force);
        if (query->name.find("move")==0) {
            widths.assign(traj.n_time_steps, query->prev_gripper_width);
        } else if (query->name.find("wait")==0) {
            widths.assign(traj.n_time_steps, query->prev_gripper_width);
        } else if (query->name.find("release")==0) {
            widths.assign(traj.n_time_steps, FLAGS_gripper_open_width);
        } else if (query->name.find("grasp")==0) {
            widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);
        } else if (query->name.find("push")==0) {
            widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);
        } else if (query->name.find("throw")==0) {
            for (int j = 0; j < traj.n_time_steps; j++) {
                if (j < traj.n_time_steps/5.) {
                    widths.push_back(FLAGS_gripper_close_width);
                } else {
                    widths.push_back(FLAGS_gripper_open_width);
                }
            }
        } else {
            std::cout << "This shouldn't happen, Assigning gripper to previous state\n";
            widths.assign(traj.n_time_steps, query->prev_gripper_width);
        }
        traj.gripper_force = forces;
        traj.gripper_width = widths;

        AppendTrajectory(total_traj, traj);
    }

    lcm_.publish(FLAGS_result_channel, &total_traj);
    for (size_t i = 0; i < q_sol.size()-1; i++) {
        std::cout<<"IK Results"<<"\n";
        std::cout<<"q_init ";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i][j]<<" ";
        }
        std::cout<<"\nq_goal";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i+1][j]<<" ";
        }
        std::cout<<"\n";
    }
    std::cout << "--------"<<query->name<<" Trajectory Published to LCM! --------" << endl;
}

lcmt_manipulator_traj GetInfCost() {
    std::cout<<"IK FAILED, publishing Inf cost\n";
    auto msg = std::make_unique<lcmt_manipulator_traj>();
    msg->cost = std::numeric_limits<double>::infinity();
    msg->n_time_steps = 0;
    msg->dim_states = 0;
    msg->dim_torques = 0;

    return *msg;
}

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
} // namespace conveyor_belt_iiwa
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::conveyor_belt_tamp::PDDLQueryHandler handler;
    handler.Run();
    return 0;
}
