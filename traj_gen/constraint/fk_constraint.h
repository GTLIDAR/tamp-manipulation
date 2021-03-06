#pragma once

#include <cstdio>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>
#include <unordered_map>

#include "drake/solvers/constraint.h"
#include "drake/solvers/nonlinear_constraint.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"

using namespace Eigen;
using namespace std;

namespace drake{
namespace traj_gen{

using multibody::MultibodyPlant;
using multibody::Parser;
using math::RigidTransformd;
using multibody::MultibodyForces;
using multibody::ModelInstanceIndex;
using math::RollPitchYaw;

template <typename T>
class FKConstraint : public solvers::NonlinearConstraint<T> {
 public: 
  FKConstraint(const drake::multibody::MultibodyPlant<T>& plant,
               const Eigen::VectorXd& target,
               const std::string& model_name,
               const std::string& frame_name,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               const std::string& description = "");

  ~FKConstraint() override {}

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const Eigen::VectorXd target_;
  const multibody::ModelInstanceIndex model_instance_;
  const multibody::ModelInstanceIndex iiwa_model_;
  const std::string frame_name_;
  std::unique_ptr<drake::systems::Context<T>> context_;

};

template <typename T>
class FKConstraint_z : public solvers::NonlinearConstraint<T> {
 public: 
  FKConstraint_z(const drake::multibody::MultibodyPlant<T>& plant,
               const std::string& model_name,
               const std::string& frame_name,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               const std::string& description = "");

  ~FKConstraint_z() override {}

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const multibody::ModelInstanceIndex model_instance_;
  const multibody::ModelInstanceIndex iiwa_model_;
  const std::string frame_name_;
  std::unique_ptr<drake::systems::Context<T>> context_;

};

template <typename T>
class FKConstraint_y : public solvers::NonlinearConstraint<T> {
 public: 
  FKConstraint_y(const drake::multibody::MultibodyPlant<T>& plant,
               const std::string& model_name,
               const std::string& frame_name,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               const std::string& description = "");

  ~FKConstraint_y() override {}

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const multibody::ModelInstanceIndex model_instance_;
  const multibody::ModelInstanceIndex iiwa_model_;
  const std::string frame_name_;
  std::unique_ptr<drake::systems::Context<T>> context_;

};

template <typename T>
class FKConstraint_x : public solvers::NonlinearConstraint<T> {
 public: 
  FKConstraint_x(const drake::multibody::MultibodyPlant<T>& plant,
               const std::string& model_name,
               const std::string& frame_name,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               const std::string& description = "");

  ~FKConstraint_x() override {}

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const multibody::ModelInstanceIndex model_instance_;
  const multibody::ModelInstanceIndex iiwa_model_;
  const std::string frame_name_;
  std::unique_ptr<drake::systems::Context<T>> context_;

};

} // traj_gen
} // drake



