/// Repackaged from runddp.cc into a library to be called from another file

#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_ddp_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/geometry/scene_graph_inspector.h"

#include "drake/lcmt_generic_string_msg.hpp"

#include "drake/traj_gen/config.h"
//#include "drake/traj_gen/spline.h"
#include "drake/traj_gen/ilqrsolver.h"
#include "drake/traj_gen/kuka_arm.h"

#include "drake/lcmt_motion_plan_query.hpp"

using namespace std;
using namespace Eigen;

#define useILQRSolver 1
#define useUDPSolver 0

/* DDP trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {


using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialMatrix PPMatrix;

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using multibody::ModelInstanceIndex;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyForces;

class DDPRunner {
public:
lcmt_ddp_traj RunUDP(stateVec_t xinit, stateVec_t xgoal, 
    const lcmt_motion_plan_query* query);
void saveVector(const Eigen::MatrixXd & _vec, const char * _name);
void saveValue(double _value, const char * _name);
void clean_file(const char * _file_name, std::string & _ret_file);

private:

//UDP parameters
stateVecTab_t joint_state_traj;
commandVecTab_t torque_traj;
stateVecTab_t joint_state_traj_interp;
commandVecTab_t torque_traj_interp;
};

} // namespace kuka_iiwa_arm
} // examples
} // drake