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
#include "drake/lcmt_motion_plan_query.hpp"
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
lcmt_motion_plan_query current_query_;
lcmt_manipulator_traj traj_;

void HandleQuery(
    const real_lcm::ReceiveBuffer*,
    const std::string&,
    const lcmt_motion_plan_query* query
) {
    current_query_ = *query;
    std::cout<<"Received "<<query->name<<"\n";

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
    bool ik_feasible;
    if (ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)) {
        q_goal = ik_res.back();
        ik_feasible = true;
    } else {
        std::cout<<"IK infeasible\n";
        ik_feasible = false;
    }

    std::cout<<"\n";
    if (!ik_feasible) {
        traj_ = GetInfCost();
        lcm_.publish(FLAGS_result_channel, &traj_);
        return;
    } else if (query->level==1) {
        std::cout<<"IK Results"<<"\n";
        for (int i = 0; i < kNumJoints; i++) {
            std::cout<<q_goal[i]<<" ";
        }
        traj_ = GetDDPRes(iiwa_q, q_goal, query);
    } else if (query->level==2) {
        std::cout<<"IK Results"<<"\n";
        for (int i = 0; i < kNumJoints; i++) {
            std::cout<<q_goal[i]<<" ";
        }
        traj_ = GetADMMRes(iiwa_q, q_goal, query);
    }

    std::vector<double> widths;
    std::vector<double> forces;
    forces.assign(traj_.n_time_steps, query->gripper_force);
    // add gripper width
    if (query->name.find("move")==0) {
        widths.assign(traj_.n_time_steps, query->prev_gripper_width);
    } else if (query->name.find("wait")==0) {
        widths.assign(traj_.n_time_steps, query->prev_gripper_width);
    } else if (query->name.find("release")==0) {
        widths.assign(traj_.n_time_steps, FLAGS_gripper_open_width);
    } else if (query->name.find("grasp")==0) {
        widths.assign(traj_.n_time_steps, FLAGS_gripper_close_width);
    } else if (query->name.find("push")==0) {
        widths.assign(traj_.n_time_steps, FLAGS_gripper_close_width);
    } else if (query->name.find("throw")==0) {
        for (int i = 0; i < traj_.n_time_steps; i++) {
            if (i < traj_.n_time_steps/5.) {
                widths.push_back(FLAGS_gripper_close_width);
            } else {
                widths.push_back(FLAGS_gripper_open_width);
            }
        }
    } else {
        std::cout << "This shouldn't happen, Assigning gripper to previous state\n";
        widths.assign(traj_.n_time_steps, query->prev_gripper_width);
    }

    traj_.gripper_force = forces;
    traj_.gripper_width = widths;

    // add wait time in traj
    if (query->wait_time) {
        int wait_time_steps = query->wait_time / query->time_step;
        traj_.n_time_steps += wait_time_steps;
        double time_step = query->time_step;
        for (int i = 0; i < wait_time_steps; i++) {
            traj_.times_sec.push_back(traj_.times_sec.back()+time_step);
            traj_.states.push_back(traj_.states.back());
            traj_.torques.push_back(traj_.torques.back());
            traj_.gripper_width.push_back(traj_.gripper_width.back());
            traj_.gripper_force.push_back(traj_.gripper_force.back());
        }
        std::cout<<"wait traj added.\n";
    }

    lcm_.publish(FLAGS_result_channel, &traj_);
    std::cout << "--------"<<query->name<<" Trajectory Published to LCM! --------" << endl;
    ClearTrajectory(traj_);
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
    return runner.RunDDP(qv_init, q_goal, query->time_horizon, query->time_step);
}

lcmt_manipulator_traj GetADMMRes(VectorXd q_init, VectorXd q_goal,
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
    return runner.RunADMM(qv_init, q_goal, query->time_horizon, query->time_step, query->name);
}

void ClearTrajectory(lcmt_manipulator_traj &traj) {
    traj.n_time_steps = 0;
    traj.dim_torques = 0;
    traj.dim_states = 0;
    traj.times_sec.clear();
    traj.states.clear();
    traj.torques.clear();
    traj.gripper_width.clear();
    traj.gripper_force.clear();
    traj.cost = 0;
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
