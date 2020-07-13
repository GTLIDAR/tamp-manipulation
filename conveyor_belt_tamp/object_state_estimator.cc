#include "drake/conveyor_belt_tamp/object_state_estimator.h"

namespace drake {
namespace conveyor_belt_tamp {
namespace manipulation_station {

template <typename T>
ObjectStateEstimator<T>::ObjectStateEstimator(
    multibody::MultibodyPlant<T>* plant,
    std::vector<multibody::ModelInstanceIndex>* object_ids)
    : plant_(plant), object_ids_(*object_ids) {

    for (size_t i = 0; i < object_ids_.size(); i++) {
        int input_port_id = this->DeclareInputPort(
            systems::kUseDefaultName,
            systems::kVectorValued,
            plant_->num_positions(object_ids_[i]) + plant_->num_velocities(object_ids_[i])
        ).get_index();

        port_id_map_.insert(
            std::pair<ModelInstanceIndex, int>(object_ids_[i], input_port_id)
        );
    }

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

    for (size_t i = 0; i < object_ids_.size(); i++) {
        const systems::BasicVector<T>* state =
            this->EvalVectorInput(
                context, port_id_map_.find(object_ids_[i])->second);

        // copy data of q and v into std vectors and push to msg
        VectorX<T> q = state->get_value().head(plant_->num_positions(object_ids_[i]));
        std::vector<double> q_std;
        q_std.resize(q.size());
        VectorX<T>::Map(&q_std[0], q.size()) = q;
        msg.q[i] = q_std;

        VectorX<T> v = state->get_value().tail(plant_->num_velocities(object_ids_[i]));
        std::vector<double> v_std;
        v_std.resize(v.size());
        VectorX<T>::Map(&v_std[0], v.size());
        msg.v[i] = v_std;

        msg.ids[i] = object_ids_[i];
    }

}

template <typename T>
lcmt_combined_object_state ObjectStateEstimator<T>::MakeOutputMsg() const {
    lcmt_combined_object_state msg;
    msg.num_objects = object_ids_.size();
    msg.q_dim = plant_->num_positions(object_ids_[0]);
    msg.v_dim = plant_->num_velocities(object_ids_[0]);
    msg.q.resize(msg.num_objects);
    msg.v.resize(msg.num_objects);
    msg.ids.resize(msg.num_objects);

    return msg;
}

template <typename T>
const systems::InputPort<T>&
    ObjectStateEstimator<T>::get_input_port_states(ModelInstanceIndex id) const {
    return this->get_input_port(port_id_map_.find(id)->second);
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
