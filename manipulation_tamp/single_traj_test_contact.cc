#include <iostream>
#include <vector>

#include "lcm/lcm-cpp.hpp"
#include "gflags/gflags.h"

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/quaternion.h"
#include "drake/traj_gen/admm_runner.h"
#include "drake/traj_gen/ddp_runner.h"
#include "drake/traj_gen/ilqr_kkt/ddp_runner_contact.h"
#include "drake/traj_gen/ilqr_kkt/admm_runner_contact.h"
#include "drake/traj_gen/admm_contact_constraints/ddp_runner_contact_new.h"
#include "drake/traj_gen/admm_contact_constraints/admm_runner_contact_new.h"

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

using namespace Eigen;
using namespace std;

using lcm::LCM;
using drake::manipulation::planner::ConstraintRelaxingIk;
using drake::traj_gen::kuka_iiwa_arm::ADMMRunner;
using drake::traj_gen::kuka_iiwa_arm::DDPRunner;
using drake::traj_gen::kuka_iiwa_arm::ADMM_KKTRunner;
using drake::traj_gen::kuka_iiwa_arm::DDP_KKTRunner;
using drake::traj_gen::kuka_iiwa_arm::ADMM_KKTRunner_new;
using drake::traj_gen::kuka_iiwa_arm::DDP_KKTRunner_new;
using drake::manipulation::kuka_iiwa::kIiwaArmNumJoints;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::traj_gen::kuka_iiwa_arm::fullstateVec_t;

namespace drake {
namespace manipulation_tamp {

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
// const lcmt_motion_plan_query* current_query_;
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

    DDP_KKTRunner_new runner;
    auto return_ptr =  runner.RunDDP_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
    runner.RunVisualizer(0.15);
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

    ADMM_KKTRunner_new runner;
    auto return_ptr = runner.RunADMM_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
    runner.RunVisualizer(0.15);
    return return_ptr;
}

};

VectorXd get_q_object_final(VectorXd q_obj_init, RigidTransformd EE_initial, RigidTransformd EE_final){
    Vector3d xyz_obj = q_obj_init.bottomRows(3);
    Vector4d qua_obj_init = q_obj_init.topRows(4);
    Quaternion<double> quaternion_obj(qua_obj_init(0), qua_obj_init(1), qua_obj_init(2), qua_obj_init(3));
    RotationMatrix<double> rot_obj(quaternion_obj);

    Vector3d xyz_ee_init = EE_initial.translation();
    RotationMatrix<double> rot_ee_init = EE_initial.rotation();

    Vector3d xyz_ee_final = EE_final.translation();
    RotationMatrix<double> rot_ee_final = EE_final.rotation();

    Vector3d xyz_ee_obj_init = xyz_obj - xyz_ee_init;
    RotationMatrix<double> rot_ee_init_final = rot_ee_init * rot_ee_final.transpose();
    Vector3d xyz_ee_obj_final = rot_ee_init_final * xyz_ee_obj_init;
    Vector3d xyz_obj_final = xyz_ee_final + xyz_ee_obj_final;

    RotationMatrix<double> rot_obj_final = rot_ee_init_final * rot_obj;
    Vector4d quaternion_obj_final = rot_obj_final.ToQuaternionAsVector4();
    VectorXd q_obj_final(7);
    q_obj_final.topRows(4) = quaternion_obj_final;
    q_obj_final.bottomRows(3) = xyz_obj_final;
    return q_obj_final;
}

int do_main() {
    drake::manipulation_tamp::TrajTestRunner runner;

    // double prev_q[] = {1.84849, 1.30959, -0.0757701, -1.37273, -1.29295, 1.59139, 2.68207}; //pushing
    
    // for pushing
    // query.desired_ee[0] = 0.5;
    // query.desired_ee[1] = 0.55;
    // query.desired_ee[2] = 0.0816099;
    // query.desired_ee[3] = 1.42092e-12;
    // query.desired_ee[4] = 0.0292037;
    // query.desired_ee[5] = 4.26875e-12;
    
    fullstateVec_t xinit,xgoal;
    double time_horizon = 0.05;
    double time_step = 0.005;
    double realtime_rate = 0.05;
    std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string action = "move";
    std::vector<Eigen::VectorXd> ik_res;
    RigidTransformd EE_init, EE_final;
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
        EE_init = RigidTransformd(rpy0, xyz0);
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

        EE_final = RigidTransformd(rpy1, xyz1);
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
            0.30

            // (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            // -0.16,
            // 0.09
            
            // (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            // 0.32,
            // 0.09
            // 0.605, -0.19499999999999995, 0.30000000000000004
            // 0.5800000000000001, -0.1, 0.275
        );
        const math::RollPitchYaw<double> rpy0(
            0,
            1.57079632679,
            1.57079632679

            // 0,
            // 0,
            // 1.57079632679

            // 0,
            // 0,
            // -1.57079632679
            // 0.0, 0.0, -1.5708
            // 0.0, 1.5708, 0.0
        );
        // rpy0.To
        EE_init = RigidTransformd(rpy0, xyz0);
        wp0.pose.set_translation(xyz0);
        wp0.pose.set_rotation(rpy0);
        wp0.constrain_orientation = true;
        wp_vec.push_back(wp0);

        // waypoint (1)
        ConstraintRelaxingIk::IkCartesianWaypoint wp1;
        const Eigen::Vector3d xyz1(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            0.0,
            0.35
            // 0.03, -0.47, 0.25
            // 0.75, 0.0, 0.45
        );
        const math::RollPitchYaw<double> rpy1(
            // 0.0, 0.0, 0.0
            0.0,
            1.57079632679,
            1.57079632679

            // 1.57079632679,
            // 0,
            // -1.57079632679

            // 0.707,
            // 0.707
            // 0.0, 1.57, -1.57
        );

        EE_final = RigidTransformd(rpy1, xyz1);
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
    xinit.setZero();
    xinit.topRows(13) << 
    // 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    // 1, 0, 0, 0,
    // 0.5800000000000001, -0.1, 0.1, 0, 0, 0, 0, 0, 0;
    1, 0, 0, 0,
    (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0, 0.09, 0, 0, 0, 0, 0, 0;
    // 1.0, 0.0, 0.0, 0.0, 0.605, -0.33, 0.30000000000000004, 0, 0, 0, 0, 0, 0;

    VectorXd q_obj_final = get_q_object_final(xinit.topRows(7), EE_init, EE_final);
    cout << q_obj_final << endl;
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // 0, 0, 0.8, 0, 0, 0, 0, 0, 0;
    // xinit.middleRows<7>(13) << 0, 0, 0, 0, 0, 0, 0;
    xinit.middleRows<7>(13) = ik_res[1];

    xgoal.setZero();
    xgoal.topRows(7) = q_obj_final;
    // xgoal.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0.3, 0.29, 0, 0, 0, 0, 0, 0;

    // xgoal.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x+0.3, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    xgoal.middleRows<7>(13) = ik_res[2];

    runner.Run(xinit, xgoal, time_horizon, time_step, realtime_rate, action);

    return 0;
}
} // namespace manipulation_tamp
} // namespace drake

int main(){
    return drake::manipulation_tamp::do_main();
}