/// Repackaged from runddp.cc into a library to be called from another file

#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_manipulator_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

#include "drake/lcmt_generic_string_msg.hpp"

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/ilqrsolver.h"
#include "drake/traj_gen/kuka_arm.h"

#include "drake/lcmt_motion_plan_query.hpp"

using namespace std;
using namespace Eigen;
using lcm::LCM;

#define useILQRSolver 1
#define useUDPSolver 0

/* DDP trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using multibody::ModelInstanceIndex;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyForces;

class DDPRunner {
public:
lcmt_manipulator_traj RunDDP(stateVec_t xinit, stateVec_t xgoal,
    double time_horizon, double time_step);
void RunVisualizer(double realtime_rate);
void saveVector(const Eigen::MatrixXd & _vec, const char * _name);
void saveValue(double _value, const char * _name);
void clean_file(const char * _file_name, std::string & _ret_file);

void HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time);

//parameters
LCM lcm_;
lcmt_robot_time robot_time_;
bool plan_finished_;
unsigned int step_;
double time_step_;
unsigned int N;
stateVecTab_t joint_state_traj;
commandVecTab_t torque_traj;
stateVecTab_t joint_state_traj_interp;
commandVecTab_t torque_traj_interp;
stateVecTab_half_t position_traj_interp;
};

} // namespace kuka_iiwa_arm
} // examples
} // drake
