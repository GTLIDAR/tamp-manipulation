#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

constexpr int kObjectNumJoints = 7;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_object_max_joint_velocities();

extern const double kObjectLcmStatusPeriod;

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake