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
    const std::string& model_name,
    const std::string& frame_name,
    const Eigen::VectorXd& lb, 
    const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          3, plant.num_positions()-2, lb, ub,
          description.empty() ? "_kuka_fk_constraint" : description),
      plant_(plant),
      model_instance_(plant_.GetModelInstanceByName(model_name)),
      frame_name_(frame_name),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void FKConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  plant_.SetPositions(context_.get(), model_instance_, x);
  *y = plant_.CalcRelativeTransform(*context_, plant_.world_frame(), plant_.GetFrameByName(frame_name_, model_instance_)).translation();

//   drake::VectorX<T> pt(3);
//   this->plant_.CalcPointsPositions(*context_, body_.body_frame(),
//                                    point_wrt_body_, plant_.world_frame(), &pt);
//   *y = dir_ * pt;
};

template class FKConstraint<double>;
template class FKConstraint<AutoDiffXd>;

} //traj_gen
} //drake