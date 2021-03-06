#include "drake/traj_gen/constraint/fk_constraint.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake{
namespace traj_gen{

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;

template <typename T>
FKConstraint<T>::FKConstraint(
    const drake::multibody::MultibodyPlant<T>& plant,
    const Eigen::VectorXd& target,
    const std::string& model_name,
    const std::string& frame_name,
    const Eigen::VectorXd& lb, 
    const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          1, plant.num_positions()-2, lb, ub,
          description.empty() ? "_kuka_fk_constraint" : description),
      plant_(plant),
      target_(target),
      model_instance_(plant_.GetModelInstanceByName(model_name)),
      iiwa_model_(plant_.GetModelInstanceByName("iiwa")),
      frame_name_(frame_name),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void FKConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  plant_.SetPositions(context_.get(), iiwa_model_, x);
  drake::VectorX<T> pt(3);
  pt = plant_.CalcRelativeTransform(*context_, plant_.world_frame(), plant_.GetFrameByName(frame_name_, model_instance_)).translation();
  *y = drake::Vector1<T>((pt-target_).norm());

//   drake::VectorX<T> pt(3);
//   this->plant_.CalcPointsPositions(*context_, body_.body_frame(),
//                                    point_wrt_body_, plant_.world_frame(), &pt);
//   *y = dir_ * pt;
};

template <typename T>
FKConstraint_z<T>::FKConstraint_z(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& model_name,
    const std::string& frame_name,
    const Eigen::VectorXd& lb, 
    const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          1, plant.num_positions()-2, lb, ub,
          description.empty() ? "_kuka_fk_constraint" : description),
      plant_(plant),
      model_instance_(plant_.GetModelInstanceByName(model_name)),
      iiwa_model_(plant_.GetModelInstanceByName("iiwa")),
      frame_name_(frame_name),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void FKConstraint_z<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  plant_.SetPositions(context_.get(), iiwa_model_, x);
  drake::VectorX<T> pt(3);
  pt = plant_.CalcRelativeTransform(*context_, plant_.world_frame(), plant_.GetFrameByName(frame_name_, model_instance_)).translation();
  *y = drake::Vector1<T>(pt[2]); // get z direction

};

template <typename T>
FKConstraint_y<T>::FKConstraint_y(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& model_name,
    const std::string& frame_name,
    const Eigen::VectorXd& lb, 
    const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          1, plant.num_positions()-2, lb, ub,
          description.empty() ? "_kuka_fk_constraint" : description),
      plant_(plant),
      model_instance_(plant_.GetModelInstanceByName(model_name)),
      iiwa_model_(plant_.GetModelInstanceByName("iiwa")),
      frame_name_(frame_name),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void FKConstraint_y<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  plant_.SetPositions(context_.get(), iiwa_model_, x);
  drake::VectorX<T> pt(3);
  pt = plant_.CalcRelativeTransform(*context_, plant_.world_frame(), plant_.GetFrameByName(frame_name_, model_instance_)).translation();
  *y = drake::Vector1<T>(pt[1]); // get z direction

};

template <typename T>
FKConstraint_x<T>::FKConstraint_x(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& model_name,
    const std::string& frame_name,
    const Eigen::VectorXd& lb, 
    const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          1, plant.num_positions()-2, lb, ub,
          description.empty() ? "_kuka_fk_constraint" : description),
      plant_(plant),
      model_instance_(plant_.GetModelInstanceByName(model_name)),
      iiwa_model_(plant_.GetModelInstanceByName("iiwa")),
      frame_name_(frame_name),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void FKConstraint_x<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  plant_.SetPositions(context_.get(), iiwa_model_, x);
  drake::VectorX<T> pt(3);
  pt = plant_.CalcRelativeTransform(*context_, plant_.world_frame(), plant_.GetFrameByName(frame_name_, model_instance_)).translation();
  *y = drake::Vector1<T>(pt[0]); // get z direction

};

template class FKConstraint<double>;
template class FKConstraint<AutoDiffXd>;
template class FKConstraint_z<double>;
template class FKConstraint_z<AutoDiffXd>;
template class FKConstraint_y<double>;
template class FKConstraint_y<AutoDiffXd>;
template class FKConstraint_x<double>;
template class FKConstraint_x<AutoDiffXd>;

} //traj_gen
} //drake