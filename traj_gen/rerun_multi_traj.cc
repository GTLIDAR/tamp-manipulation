#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_manipulator_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

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

/* ADMM trajectory generation */

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const int32_t kNumJoints = 7;


class ReplayTraj {
  public:
  void Run() {
    double time_horizon_ = 1.0;
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
    _file_name += "position_concatenated_DDP"; //"joint_trajectory_ADMM";
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

int do_main() {
  ReplayTraj runner;
  runner.Run();

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::traj_gen::kuka_iiwa_arm::do_main();
}