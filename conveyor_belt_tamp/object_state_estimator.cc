#include "drake/conveyor_belt_tamp/object_state_estimator.h"

namespace drake {
namespace conveyor_belt_tamp {
namespace manipulation_station {

template <typename T>
ObjectStateEstimator<T>::ObjectStateEstimator(
    multibody::MultibodyPlant<T>* plant,
    std::vector<BodyIndex>* object_ids)
    : plant_(plant), object_ids_(*object_ids) {

    input_port_body_poses_ = this->DeclareAbstractInputPort(
        "input_port_body_poses",
        Value<std::vector<math::RigidTransform<double>>>()
    ).get_index();

    input_port_body_velocities_ = this->DeclareAbstractInputPort(
        "input_port_body_velocities",
        Value<std::vector<SpatialVelocity<double>>>()
    ).get_index();

    output_port_index_msg_ = this->DeclareAbstractOutputPort(
        "output_port_object_state",
        &ObjectStateEstimator::MakeOutputMsg,
        &ObjectStateEstimator::OutputObjectState
    ).get_index();
}

template <typename T>
void ObjectStateEstimator<T>::OutputObjectState(
    const systems::Context<T>& context,
    lcmt_combined_object_state* output) const {

    lcmt_combined_object_state& msg = *output;

    msg.utime = context.get_time()*1e6;

    const auto& X_WB_all = this->get_input_port_body_poses()
      .template Eval<std::vector<math::RigidTransform<double>>>(context);

    const auto& V_WB_all = this->get_input_port_body_velocities()
      .template Eval<std::vector<SpatialVelocity<double>>>(context);

    for (size_t i = 0; i < object_ids_.size(); i++) {
        const math::RigidTransform<double>& X_WB = X_WB_all[object_ids_[i]];
        const VectorX<double>& xyz = X_WB.translation();
        const auto& rpy_object = math::RollPitchYaw<double>(X_WB.rotation());
        const VectorX<double>& rpy = rpy_object.vector();

        std::vector<double> q;
        q.resize(msg.q_dim, 0.);
        VectorX<T>::Map(&q[0], xyz.size()) = xyz;
        VectorX<T>::Map(&q[3], rpy.size()) = rpy;
        msg.q[i] = q;

        const SpatialVelocity<double>& V_WB = V_WB_all[object_ids_[i]];

        std::vector<double> v;
        v.resize(msg.v_dim, 0.);
        VectorX<T>::Map(&v[0], 3) = V_WB.translational();
        VectorX<T>::Map(&v[3], 3) = V_WB.rotational();
        msg.v[i] = v;
    }

}

template <typename T>
lcmt_combined_object_state ObjectStateEstimator<T>::MakeOutputMsg() const {
    lcmt_combined_object_state msg;
    msg.num_objects = object_ids_.size();
    msg.q_dim = 6;
    msg.v_dim = 6;
    msg.q.resize(msg.num_objects);
    msg.v.resize(msg.num_objects);
    msg.ids.resize(msg.num_objects);

    return msg;
}

// template <typename T>
// const systems::InputPort<T>&
//     ObjectStateEstimator<T>::get_input_port_states(ModelInstanceIndex id) const {
//     return this->get_input_port(port_id_map_.find(id)->second);
// }

template <typename T>
const systems::InputPort<T>&
    ObjectStateEstimator<T>::get_input_port_body_poses() const {
    return this->get_input_port(input_port_body_poses_);
}

template <typename T>
const systems::InputPort<T>&
    ObjectStateEstimator<T>::get_input_port_body_velocities() const {
    return this->get_input_port(input_port_body_velocities_);
}

template <typename T>
const systems::OutputPort<T>&
    ObjectStateEstimator<T>::get_output_port_msg() const {
    return this->get_output_port(output_port_index_msg_);
}

template class ObjectStateEstimator<double>;

} // namespace manipulation_station
} // namespace conveyor_belt_tamp
} // namespace drake
