#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_manipulator_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

#include "drake/lcmt_generic_string_msg.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

/* ADMM trajectory generation */
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>

#include "drake/traj_gen/config.h"

using namespace std;
using namespace Eigen;
using lcm::LCM;

/* ADMM trajectory generation */

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const int32_t kNumJoints = 7;
using manipulation::kuka_iiwa::kIiwaArmNumJoints;

class ReplayTraj {
  public:
  void Run() {
    double time_horizon_ = 2.0;
    double time_step_ = 0.005;
    // int NumberofKnotPt = 2000;
    int NumberofKnotPt = int(time_horizon_/time_step_);
    unsigned int N = 3*(NumberofKnotPt*InterpolationScale);
    stateVecTab_half_t x_display;
    x_display.resize(N);

    //======================================================================
    //======================================================================
    // (Test only) Reading Saved Trajectory
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += "position_concatenated_DDP_move"; //"joint_trajectory_ADMM";
    _file_name += ".csv";
    ifstream myfile(_file_name);
    if(!myfile) 
    {
        cerr << "Couldn't open file " << _file_name << endl;
    }

    string line;
    unsigned int kk_p=0;
    while( getline(myfile, line) )
    {
        // std::cout << "read csv: " << line << std::endl;
        
        stringstream s;
        double db;
        
        s << line;

        for (unsigned int kk=0; kk<7; kk++) {
            s >> db;
            // std::cout << "db: " << db << std::endl;
            x_display[kk_p](kk,0) = db;
            
        }
        kk_p++;
    }

    //======================================================================
    //======================================================================
    position_traj_interp.resize(N);
    for(unsigned int i=0;i<N;i++){
      position_traj_interp[i] = x_display[i];
    }

    // texec /= Num_run;


    // Send over points using LCM
    // need this for dynamic memory allocation (push_back)
    auto ptr = std::make_unique<lcmt_manipulator_traj>();
    
    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = N; 
    ptr->cost = 0;

      //============================================

    for (int32_t i=0; i < ptr->n_time_steps; ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(time_step_*i/InterpolationScale));

      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=0; j < ptr->dim_states; ++j) { 
        ptr2->push_back(0);
        ptr2_st->push_back(position_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
      ptr->gripper_force.push_back(100.0);

      // if (i < 3*NumberofKnotPt*InterpolationScale-500) { // at this point *10 cuz interpolated
      //   ptr->gripper_width.push_back(0.0);
      // } 
      // else {
      //   ptr->gripper_width.push_back(100.0);
      // }
    //   if (i < NumberofKnotPt*InterpolationScale+1000) { // at this point *10 cuz interpolated
    //     ptr->gripper_width.push_back(100.0);
    //   } 
    //   else if (i >= NumberofKnotPt*InterpolationScale+1000 && i < 2*NumberofKnotPt*InterpolationScale){
    //     ptr->gripper_width.push_back(30.0);
    //   }

    //   else if (i >= 2*NumberofKnotPt*InterpolationScale && i < 3*NumberofKnotPt*InterpolationScale-500) { // at this point *10 cuz interpolated
    //     ptr->gripper_width.push_back(30.0);
    //   } 

    //   else {
    //     ptr->gripper_width.push_back(100.0);
    //   }

      if (i < NumberofKnotPt*InterpolationScale+1000) { // at this point *10 cuz interpolated
        ptr->gripper_width.push_back(50.0);
      } 
      else if (i >= NumberofKnotPt*InterpolationScale+1000 && i < 2*NumberofKnotPt*InterpolationScale){
        ptr->gripper_width.push_back(50.0);
      }

      else if (i >= 2*NumberofKnotPt*InterpolationScale && i < 3*NumberofKnotPt*InterpolationScale-500) { // at this point *10 cuz interpolated
        ptr->gripper_width.push_back(50.0);
      } 

      else {
        ptr->gripper_width.push_back(50.0);
      }

    }

    // need this because...?
    ddp_traj_ = *ptr;

    lcm_.publish(kLcmPlanChannel, &ddp_traj_);
    cout << "-------- Replayed Trajectory Published to LCM! --------" << endl;
  }

 private:
  lcm::LCM lcm_;
  lcmt_manipulator_traj ddp_traj_;

  //UDP parameters
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;
  stateVecTab_half_t position_traj_interp;

};

class ReplayTrajVisualizer {
  public:
  void Run(double realtime_rate) {
    double time_horizon_ = 1.0;
    time_step_ = 0.01;
    // int NumberofKnotPt = 2000;
    N = int(1*((time_horizon_/time_step_)*InterpolationScale));
    stateVecTab_half_t x_display;
    x_display.resize(N+1);

    //======================================================================
    //======================================================================
    // (Test only) Reading Saved Trajectory
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += "joint_trajectory_interpolated_ADMM_demo"; //"joint_trajectory_ADMM";
    // _file_name += "xnew_ca_interpolated_ADMM_demo"; //"joint_trajectory_ADMM";
    _file_name += ".csv";
    ifstream myfile(_file_name);
    if(!myfile) 
    {
        cerr << "Couldn't open file " << _file_name << endl;
    }

    string line;
    unsigned int kk_p=0;
    while( getline(myfile, line) )
    {
        // std::cout << "read csv: " << line << std::endl;
        
        stringstream s;
        double db;
        
        s << line;

        for (unsigned int kk=0; kk<7; kk++) {
            s >> db;
            // std::cout << "db: " << db << std::endl;
            x_display[kk_p](kk,0) = db;
            
        }
        kk_p++;
    }

    //======================================================================
    //======================================================================
    // position_traj_interp.resize(N);
    // for(unsigned int i=0;i<N;i++){
    //   position_traj_interp[i] = x_display[i];
    // }

    // texec /= Num_run;
    lcm_.subscribe(kLcmTimeChannel,
                    &ReplayTrajVisualizer::HandleRobotTime, this);
    lcmt_iiwa_status iiwa_state;
    lcmt_schunk_wsg_status wsg_status;
    lcmt_object_status object_state;
    iiwa_state.num_joints = kIiwaArmNumJoints;
    object_state.num_joints = 7;
    iiwa_state.joint_position_measured.resize(kIiwaArmNumJoints, 0.);   
    iiwa_state.joint_velocity_estimated.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_position_commanded.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_position_ipo.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_measured.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_commanded.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_external.resize(kIiwaArmNumJoints, 0.);
    
    //////////////// !!!!!!!!!! /////////////////////
    // Warning: just for visualization since the gripper_position_output_port has been depricated in latest drake update
    // actual_position_mm = finger1 -- negative
    // actual_speed_mm_per_s = finger2 -- positive
    // maximum width = 110

    wsg_status.actual_position_mm = -25;
    wsg_status.actual_speed_mm_per_s = 25;

    object_state.joint_position_measured.resize(7, 0.);   
    object_state.joint_velocity_estimated.resize(7, 0.);
    object_state.joint_position_commanded.resize(7, 0.);
    object_state.joint_position_ipo.resize(7, 0.);
    object_state.joint_torque_measured.resize(7, 0.);
    object_state.joint_torque_commanded.resize(7, 0.);
    object_state.joint_torque_external.resize(7, 0.);

    VectorXd obstacle_pose(7);
    obstacle_pose <<  1.0, 0.0, 0.0, 0.0, 0.6, 0.05, 0.1;
    for (int joint = 0; joint < 7; joint++) 
    {
      object_state.joint_position_measured[joint] = obstacle_pose[joint];
    }
    
    drake::log()->info("Publishing trajectory to visualizer");
    plan_finished_ = false;

    while(true){
        while (0 == lcm_.handleTimeout(100) || iiwa_state.utime == -1 
        || plan_finished_) { }

        // if(!start_publish){
        //   start_time = robot_time_.utime;     
        //   cout << "start_time: " << start_time << endl; 
        //   start_publish = true;
        // }

        // Update status time to simulation time
        // Note: utime is in microseconds
        iiwa_state.utime = robot_time_.utime;
        wsg_status.utime = robot_time_.utime;
        // step_ = int((robot_time_.utime / 1000)*(kIiwaLcmStatusPeriod/(time_step/InterpolationScale)));
        step_ = int(((robot_time_.utime) / 1000)*(0.001*realtime_rate/(time_step_/InterpolationScale)));
        // cout << step_ << endl;

        if(step_ >= N+1)
        {
            drake::log()->info("Interpolated trajectory has been published");
            plan_finished_ = true;
            break;
        }

        // pass the interpolated traj to lcm
        for (int32_t j=0; j < iiwa_state.num_joints; ++j) { 
            iiwa_state.joint_position_measured[j] = x_display[step_][j];
        }

        lcm_.publish(kLcmStatusChannel, &iiwa_state);
        lcm_.publish(kLcmObjectStatusChannel, &object_state);
        lcm_.publish(kLcmSchunkStatusChannel, &wsg_status);
    }

    // Send over points using LCM
    // need this for dynamic memory allocation (push_back)
  
    cout << "-------- Replayed Trajectory Published to LCM! --------" << endl;
  }

  void HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                        const lcmt_robot_time* robot_time) {
          robot_time_ = *robot_time;
  }

 private:
  lcm::LCM lcm_;
  lcmt_robot_time robot_time_;
  bool plan_finished_;
  unsigned int step_;
  double time_step_;
  unsigned int N;
  lcmt_manipulator_traj ddp_traj_;

  //UDP parameters
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;
  stateVecTab_half_t position_traj_interp;

};

int do_main() {
  // ReplayTraj runner;
  ReplayTrajVisualizer runner;
  runner.Run(0.05);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::traj_gen::kuka_iiwa_arm::do_main();
}