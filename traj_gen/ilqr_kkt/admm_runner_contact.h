/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

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
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

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
#include "drake/traj_gen/ilqr_kkt/ilqr-kkt_solver_track.h"
#include "drake/traj_gen/ilqr_kkt/kuka_arm_track_contact.h"

#include "drake/lcmt_motion_plan_query.hpp"


using namespace std;
using namespace Eigen;

/* ADMM trajectory generation */

static std::list< const char*> admm_gs_filename;
static std::list< std::string > admm_gs_filename_string;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using multibody::ModelInstanceIndex;
using multibody::MultibodyForces;
using math::RigidTransformd;
using math::RollPitchYaw;

class ADMM_KKTRunner {
  public:
  lcmt_manipulator_traj RunADMM(fullstateVec_t xinit, fullstateVec_t xgoal,
    double time_horizon, double time_step, string action_name);

  projfullStateAndCommandTab_t projection(const fullstateVecTab_t& xnew,
    const commandVecTab_t& unew, unsigned int NumberofKnotPt,
    string action_name);

  void saveVector(const Eigen::MatrixXd & _vec, const char * _name);

  void saveValue(double _value, const char * _name);

  void clean_file(const char * _file_name, std::string & _ret_file);

 private:
  // lcm::LCM lcm_;
  // lcmt_manipulator_traj ddp_traj_;

  //UDP parameters
  fullstateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  fullstateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;
  // unsigned int traj_knot_number_ = 0;
};

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake
