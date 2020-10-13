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
using drake::traj_gen::kuka_iiwa_arm::stateVecTab_half_t;

namespace drake {
namespace conveyor_belt_tamp {

class TrajTestRunner {
public:
TrajTestRunner() {
    model_path_ = FindResourceOrThrow(FLAGS_KukaIiwaUrdf);
}

void Run(double realtime_rate) {
    realtime_rate_ = realtime_rate;
    
    std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string action_name_ = "move";
    std::vector<Eigen::VectorXd> ik_res;
    if (action_name_.compare("push")==0){
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
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x+0.3,
            0.0,
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
        // Vector3d object_pos_init = xyz0;
        // object_pos_init(2, 0) -= 0.21;
        // Vector3d object_pos_goal = xyz1;
        // object_pos_goal(2, 0) -= 0.21;

    }
    // vector<std::string> fn={"initilization", "reachdown", "lift up", "steady move", "putdown"};

    double time_horizon_ = 1.0;
    double time_step_ = 0.005;
    // int NumberofKnotPt = 2000;
    int NumberofKnotPt = int(time_horizon_/time_step_);
    cout << NumberofKnotPt << endl;
    vector<std::string> fn={"initilization", "waiting", "move"};
    auto ptr = std::make_unique<lcmt_manipulator_traj>();
    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = fn.size()*(NumberofKnotPt*InterpolationScale); 
    ptr->cost = 0; 
    stateVecTab_half_t final;
    final.resize(fn.size()*(NumberofKnotPt*InterpolationScale));
    vector<stateVecTab_half_t> position_traj_interp_vec;


    fullstateVec_t xinit,xgoal;
    xinit.setZero();
    xinit.topRows(13) << 1, 0, 0, 0, 
    (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0, 0.09, 0, 0, 0, 0, 0, 0;
    
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    xinit.middleRows<7>(13) = ik_res[1];

    xgoal.setZero();
    xgoal.topRows(13) << 1, 0, 0, 0, 
    (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x+0.3, 0, 0.29, 0, 0, 0, 0, 0, 0;
    // xgoal.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.01-FLAGS_default_iiwa_x+0.3, 0.55, FLAGS_kConveyorBeltTopZInWorld-FLAGS_kTableTopZInWorld+0.09, 0, 0, 0, 0, 0, 0;
    xgoal.middleRows<7>(13) = ik_res[2];

    /////////////////////// motion generation /////////////////////////
    std::cout<<"Moving to the initial configuration\n";
    DDPRunner runner0;
    VectorXd inital_config(stateSize);
    inital_config.setZero();
    runner0.RunDDP(inital_config, xinit.bottomRows(14), time_horizon_, time_step_);
    position_traj_interp_vec.push_back(runner0.position_traj_interp);
    // runner.RunVisualizer(0.2);

    stateVecTab_half_t waiting_pos;
    waiting_pos.assign(NumberofKnotPt*InterpolationScale, runner0.position_traj_interp.back());
    position_traj_interp_vec.push_back(waiting_pos);

    // std::vector<double> widths;
    // std::vector<double> forces;
    // forces.assign(traj.n_time_steps, FLAGS_gripper_force);
    // widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);

    // traj.gripper_force = forces;
    // traj.gripper_width = widths;


    // lcm_.publish(FLAGS_plan_channel, &traj);
    // std::cout<<"Trajectory Published\n";

    // std::cout<<"IK Successful, sent to ddp_kkt\n";
    // std::cout<<"ddp initial pos: " << xinit.transpose() << std::endl;
    // std::cout<<"ddp goal pos: " << xgoal.transpose() << std::endl;

    DDP_KKTRunner runner;
    runner.RunDDP_KKT(xinit, xgoal, time_horizon_, time_step_, action_name_);
    position_traj_interp_vec.push_back(runner.position_traj_interp);

    // runner.RunVisualizer(0.2);
    // std::vector<double> widths;
    // std::vector<double> forces;
    // forces.assign(traj.n_time_steps, FLAGS_gripper_force);
    // widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);

    // traj.gripper_force = forces;
    // traj.gripper_width = widths;

    // lcm_.publish(FLAGS_plan_channel, &traj);
    // std::cout<<"Trajectory Published\n";
    // putting everything in final form
    for (unsigned int v=0; v<fn.size(); v++){
    for (int32_t i=0; i < NumberofKnotPt*InterpolationScale; i++) {
        for (int32_t j=0; j < ptr->dim_states; j++) { 
        final[v*(NumberofKnotPt*InterpolationScale) + i][j] = position_traj_interp_vec[v][i][j];
        }
    }
    }

    for(unsigned int i=0;i<fn.size()*(NumberofKnotPt*InterpolationScale);i++){
    saveVector(final[i], "position_concatenated_DDP_move");
    }

    // creating lcm message
    for (uint32_t i=0; i < fn.size()*(NumberofKnotPt*InterpolationScale); ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(time_step_*i/InterpolationScale));
      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=0; j < ptr->dim_states; ++j) { 
        ptr2->push_back(0);
        ptr2_st->push_back(final[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
      ptr->gripper_force.push_back(100.0);
      ptr->gripper_width.push_back(50.0);

      // if (fn[0] == "reachdown") {
      //   if (i < NumberofKnotPt*InterpolationScale-500) { // at this point *10 cuz interpolated
      //     ptr->gripper_width.push_back(100.0);
      //   } 
      //   else if (i >= NumberofKnotPt*InterpolationScale-500 && i < 4*NumberofKnotPt*InterpolationScale-500){
      //     ptr->gripper_width.push_back(0.0);
      //   }
      // }

      // if (fn[3] == "putdown") {
      //   if (i >= 4*NumberofKnotPt*InterpolationScale-500) { // at this point *10 cuz interpolated
      //     ptr->gripper_width.push_back(100.0);
      //   } 
      //   // else {
      //   //   ptr->gripper_width.push_back(100.0);
      //   // }
      // }
      // else {
      //   ptr->gripper_width.push_back(0.0);
      // }

    //   if (i < NumberofKnotPt*InterpolationScale-1000) { // at this point *10 cuz interpolated
    //     ptr->gripper_width.push_back(100.0);
    //   } 
    //   else if (i >= NumberofKnotPt*InterpolationScale-1000 && i < 4*NumberofKnotPt*InterpolationScale-1000){
    //     ptr->gripper_width.push_back(0.0);
    //   }

    //   else if (i >= 4*NumberofKnotPt*InterpolationScale-1000) { // at this point *10 cuz interpolated
    //     ptr->gripper_width.push_back(100.0);
    //   } 

    //   else {
    //     ptr->gripper_width.push_back(0.0);
    //   }
    }

    lcmt_manipulator_traj ddp_traj_;
    ddp_traj_ = *ptr;
    // std::cout<<"Press any key to continue...\n";
    // while (std::getc(stdin)==EOF) {}
    lcm_.publish(FLAGS_plan_channel, &ddp_traj_);
}

private:
string model_path_;
LCM lcm_;
// const lcmt_motion_plan_query* current_query_;
double realtime_rate_; 

// lcmt_manipulator_traj GetDDPRes(VectorXd q_init, VectorXd q_goal,
//     const lcmt_motion_plan_query* query) {
//     std::cout<<"IK Successful, sent to ddp\n";
//     std::cout<<"ddp initial pos: " << q_init.transpose() << std::endl;
//     std::cout<<"ddp goal pos: " << q_goal.transpose() << std::endl;

//     VectorXd qv_init;
//     qv_init = Eigen::VectorXd::Zero(stateSize);
//     VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
//     // std::cout<<"qv_init:\n"<<qv_init<<"\n";

//     VectorXd qv_goal;
//     qv_goal = VectorXd::Zero(stateSize);
//     VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
//     // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";

//     DDPRunner runner;
//     return runner.RunDDP(qv_init, qv_goal, query->time_horizon, query->time_step);
// }

// lcmt_manipulator_traj GetDDP_KKTRes(VectorXd q_init, VectorXd q_goal) {
//     std::cout<<"IK Successful, sent to ddp_kkt\n";
//     std::cout<<"ddp initial pos: " << q_init.transpose() << std::endl;
//     std::cout<<"ddp goal pos: " << q_goal.transpose() << std::endl;

//     DDP_KKTRunner runner;
//     auto return_ptr =  runner.RunDDP_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
//     runner.RunVisualizer(0.2);
//     return return_ptr;
// }

// lcmt_manipulator_traj GetADMMRes(VectorXd q_init, VectorXd q_goal,
//     const lcmt_motion_plan_query* query) {
//     std::cout<<"IK Successful, sent to admm\n";
//     std::cout<<"admm initial pos: " << q_init.transpose() << std::endl;
//     std::cout<<"admm goal pos: " << q_goal.transpose() << std::endl;

//     VectorXd qv_init;
//     qv_init = Eigen::VectorXd::Zero(stateSize);
//     VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
//     // std::cout<<"qv_init:\n"<<qv_init<<"\n";

//     VectorXd qv_goal;
//     qv_goal = VectorXd::Zero(stateSize);
//     VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
//     // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";
//     ADMMRunner runner;
//     return runner.RunADMM(qv_init, qv_goal, query->time_horizon, query->time_step, query->name);
// }

// lcmt_manipulator_traj GetADMM_KKTRes(VectorXd q_init, VectorXd q_goal) {
//     std::cout<<"IK Successful, sent to admm_kkt\n";
//     std::cout<<"admm initial pos: " << q_init.transpose() << std::endl;
//     std::cout<<"admm goal pos: " << q_goal.transpose() << std::endl;

//     ADMM_KKTRunner runner;
//     auto return_ptr = runner.RunADMM_KKT(q_init, q_goal, time_horizon_, time_step_, action_name_);
//     runner.RunVisualizer(0.2);
//     return return_ptr;
// }

void saveVector(const Eigen::MatrixXd & _vec, const char * _name) {
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    for (int i(0); i < _vec.rows(); ++i){
        save_file<<_vec(i,0)<< "\t";
    }
    save_file<<"\n";
    save_file.flush();
    save_file.close();
}

void clean_file(const char * _file_name, std::string & _ret_file){
      std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
      if(gs_fileName_string.end() == iter){
          gs_fileName_string.push_back(_file_name);
          remove(_ret_file.c_str());
      }
}
};

} // namespace tamp_conveyor_belt
} // namespace drake

int main(){
    drake::conveyor_belt_tamp::TrajTestRunner runner;
    double realtime_rate = 0.2;
    // double prev_q[] = {1.84849, 1.30959, -0.0757701, -1.37273, -1.29295, 1.59139, 2.68207}; //pushing
    
    // for pushing
    // query.desired_ee[0] = 0.5;
    // query.desired_ee[1] = 0.55;
    // query.desired_ee[2] = 0.0816099;
    // query.desired_ee[3] = 1.42092e-12;
    // query.desired_ee[4] = 0.0292037;
    // query.desired_ee[5] = 4.26875e-12;

    runner.Run(realtime_rate);

    return 0;
}