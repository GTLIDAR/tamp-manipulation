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
#include "drake/traj_gen/ilqr_kkt/ddp_runner_contact.h"
#include "drake/traj_gen/ilqr_kkt/admm_runner_contact.h"

#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/lcmt_motion_plan_query.hpp"
#include "drake/traj_gen/config.h"

DEFINE_bool(use_admm, false, "whether to use admm or ddp");

DEFINE_double(gripper_open_width, 100, "Width gripper opens to in mm");
DEFINE_double(gripper_close_width, 10, "Width gripper closes to in mm");
DEFINE_double(gripper_force, 50, "force for gripper");
DEFINE_double(table_width, 0.7112, "Width of table supporting kuka arm");
DEFINE_double(belt_width, 0.4, "Width of conveyor belt");
DEFINE_double(default_iiwa_x, 0.2, "X position of iiwa base");
DEFINE_double(kConveyorBeltTopZInWorld, 0.736 + 0.02 / 2, "height of belt");
DEFINE_double(kTableTopZInWorld, 0.736 + 0.057 / 2, "height of belt");
DEFINE_string(plan_channel, "COMMITTED_ROBOT_PLAN", "Plan channels for plan");
DEFINE_string(
    KukaIiwaUrdf,
    "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf",
    "file name of iiwa7 urdf"
);
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

using lcm::LCM;
using drake::manipulation::planner::ConstraintRelaxingIk;
using drake::traj_gen::kuka_iiwa_arm::ADMMRunner;
using drake::traj_gen::kuka_iiwa_arm::DDPRunner;
using drake::traj_gen::kuka_iiwa_arm::ADMM_KKTRunner;
using drake::traj_gen::kuka_iiwa_arm::DDP_KKTRunner;
using drake::manipulation::kuka_iiwa::kIiwaArmNumJoints;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::traj_gen::kuka_iiwa_arm::fullstateVec_t;

namespace drake {
namespace conveyor_belt_tamp {

class TrajTestRunner {
public:
TrajTestRunner() {
    model_path_ = FindResourceOrThrow(FLAGS_KukaIiwaUrdf);
}

void Run(VectorXd xinit, VectorXd xgoal, double time_horizon, double time_step, double realtime_rate, string action_name) {
    time_horizon_ = time_horizon;
    time_step_ = time_step;
    realtime_rate_ = realtime_rate;
    action_name_ = action_name;
    lcmt_manipulator_traj traj;
    if (FLAGS_use_admm) {
        traj = GetADMM_KKTRes(xinit, xgoal);
    } else {
        traj = GetDDP_KKTRes(xinit, xgoal);
    }
    std::vector<double> widths;
    std::vector<double> forces;
    forces.assign(traj.n_time_steps, FLAGS_gripper_force);
    widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);

    traj.gripper_force = forces;
    traj.gripper_width = widths;

    std::cout<<"Goal: "<< xgoal<<"\n";
    std::cout<<"Press any key to continue...\n";
    while (std::getc(stdin)==EOF) {}

    lcm_.publish(FLAGS_plan_channel, &traj);
    std::cout<<"Trajectory Published\n";
}

private:
string model_path_;
LCM lcm_;
const lcmt_motion_plan_query* current_query_;
double time_horizon_; 
double time_step_;
double realtime_rate_; 
string action_name_;

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
    return runner.RunDDP(qv_init, qv_goal, query->time_horizon, query->time_step);
}

lcmt_manipulator_traj GetDDP_KKTRes(VectorXd q_init, VectorXd q_goal) {
    std::cout<<"IK Successful, sent to ddp_kkt\n";
    std::cout<<"ddp initial pos: " << q_init.transpose() << std::endl;
    std::cout<<"ddp goal pos: " << q_goal.transpose() << std::endl;

    DDP_KKTRunner runner;
    auto return_ptr =  runner.RunDDP_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
    runner.RunVisualizer(0.2);
    return return_ptr;
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
    return runner.RunADMM(qv_init, qv_goal, query->time_horizon, query->time_step, query->name);
}

lcmt_manipulator_traj GetADMM_KKTRes(VectorXd q_init, VectorXd q_goal) {
    std::cout<<"IK Successful, sent to admm_kkt\n";
    std::cout<<"admm initial pos: " << q_init.transpose() << std::endl;
    std::cout<<"admm goal pos: " << q_goal.transpose() << std::endl;

    ADMM_KKTRunner runner;
    auto return_ptr = runner.RunADMM_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
    runner.RunVisualizer(0.2);
    return return_ptr;
}

};

int do_main() {
    drake::conveyor_belt_tamp::TrajTestRunner runner;

    // double prev_q[] = {1.84849, 1.30959, -0.0757701, -1.37273, -1.29295, 1.59139, 2.68207}; //pushing
    
    // for pushing
    // query.desired_ee[0] = 0.5;
    // query.desired_ee[1] = 0.55;
    // query.desired_ee[2] = 0.0816099;
    // query.desired_ee[3] = 1.42092e-12;
    // query.desired_ee[4] = 0.0292037;
    // query.desired_ee[5] = 4.26875e-12;
    
    fullstateVec_t xinit,xgoal;
    double time_horizon = 1.0;
    double time_step = 0.005;
    double realtime_rate = 0.2;
    std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string action = "move";
    std::vector<Eigen::VectorXd> ik_res;
    if (action.compare("push")==0){
        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
        //waypoint (0)
        ConstraintRelaxingIk::IkCartesianWaypoint wp0;
        const Eigen::Vector3d xyz0(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x-0.26,
            0.55,
            0.0816099
        );
        const math::RollPitchYaw<double> rpy0(
            // 0,
            // 1.57079632679,
            // 1.57079632679
            1.42092e-12,
            0.0292037,
            4.26875e-12
        );
        // rpy0.To
        wp0.pose.set_translation(xyz0);
        wp0.pose.set_rotation(rpy0);
        wp0.constrain_orientation = true;
        wp_vec.push_back(wp0);

        // waypoint (1)
        ConstraintRelaxingIk::IkCartesianWaypoint wp1;
        const Eigen::Vector3d xyz1(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x-0.26+0.3,
            0.55,
            0.0816099
        );
        const math::RollPitchYaw<double> rpy1(
            // 0,
            // 1.57079632679,
            // 1.57079632679
            1.42092e-12,
            0.0292037,
            4.26875e-12
        );

        wp1.pose.set_translation(xyz1);
        wp1.pose.set_rotation(rpy1);
        wp1.constrain_orientation = true;
        wp_vec.push_back(wp1);

        Eigen::VectorXd iiwa_q(7);
        iiwa_q << -0.133372, 0.251457, -0.0461879, -1.21048, 0.0324702, 0.928553, -0.190112; //warm-start for grasping from top

        ConstraintRelaxingIk ik(
            kIiwaUrdf,
            FLAGS_ee_name
        );

        if(!ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)){
        cout << "infeasible" << endl;
        }
        for (unsigned int y=0; y<ik_res.size(); y++){
        cout << ik_res[y].transpose() << endl;
        }
    }
    else{
        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
        //waypoint (0)
        ConstraintRelaxingIk::IkCartesianWaypoint wp0;
        const Eigen::Vector3d xyz0(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            0.0,
            0.3
        );
        const math::RollPitchYaw<double> rpy0(
            0,
            1.57079632679,
            1.57079632679
        );
        // rpy0.To
        wp0.pose.set_translation(xyz0);
        wp0.pose.set_rotation(rpy0);
        wp0.constrain_orientation = true;
        wp_vec.push_back(wp0);

        // waypoint (1)
        ConstraintRelaxingIk::IkCartesianWaypoint wp1;
        const Eigen::Vector3d xyz1(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            0.3,
            0.5
        );
        const math::RollPitchYaw<double> rpy1(
            0,
            1.57079632679,
            1.57079632679
        );

        wp1.pose.set_translation(xyz1);
        wp1.pose.set_rotation(rpy1);
        wp1.constrain_orientation = true;
        wp_vec.push_back(wp1);

        Eigen::VectorXd iiwa_q(7);
        iiwa_q << -0.133372, 0.251457, -0.0461879, -1.21048, 0.0324702, 0.928553, -0.190112; //warm-start for grasping from top

        ConstraintRelaxingIk ik(
            kIiwaUrdf,
            FLAGS_ee_name
        );

        if(!ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)){
        cout << "infeasible" << endl;
        }
        for (unsigned int y=0; y<ik_res.size(); y++){
        cout << ik_res[y].transpose() << endl;
        }

        // object initial and final states
        Vector3d object_pos_init = xyz0;
        object_pos_init(2, 0) -= 0.21;
        Vector3d object_pos_goal = xyz1;
        object_pos_goal(2, 0) -= 0.21;

    }
    xinit.setZero();
    xinit.topRows(13) << 1, 0, 0, 0,
    (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0, 0.09, 0, 0, 0, 0, 0, 0;
    
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // 0, 0, 0.8, 0, 0, 0, 0, 0, 0;
    // xinit.middleRows<7>(13) << 0, 0, 0, 0, 0, 0, 0;
    xinit.middleRows<7>(13) = ik_res[1];

    xgoal.setZero();
    xgoal.topRows(13) << 1, 0, 0, 0, 
    (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0.3, 0.29, 0, 0, 0, 0, 0, 0;
    // xgoal.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x+0.3, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    xgoal.middleRows<7>(13) = ik_res[2];

    runner.Run(xinit, xgoal, time_horizon, time_step, realtime_rate, action);

    return 0;
}
} // namespace tamp_conveyor_belt
} // namespace drake

int main(){
    return drake::conveyor_belt_tamp::do_main();
}