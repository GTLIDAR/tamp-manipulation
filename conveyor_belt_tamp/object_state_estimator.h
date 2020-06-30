#include <string>
#include <iostream>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcmt_combined_object_state.hpp"

namespace drake {
namespace conveyor_belt_tamp {
namespace manipulation_station {

using multibody::ModelInstanceIndex;

template <typename T>
class ObjectStateEstimator : public systems::LeafSystem<T> {
public:
explicit ObjectStateEstimator(multibody::MultibodyPlant<T>* plant, 
    std::vector<ModelInstanceIndex>* object_ids);

// calc callback function
void OutputObjectState(
    const systems::Context<T>& context, lcmt_combined_object_state* output) const;

// alloc callback function
lcmt_combined_object_state MakeOutputMsg() const;

const systems::InputPort<T>& get_input_port_states(ModelInstanceIndex id) const;

const systems::OutputPort<T>& get_output_port_msg() const;

private:
const multibody::MultibodyPlant<T>* plant_;
std::vector<ModelInstanceIndex> object_ids_;
std::map<ModelInstanceIndex, int> port_id_map_;
std::vector<int> input_port_index_object_states_;
int output_port_index_msg_{0};
};

} // namespace manipulation_station
} // namespace conveyor_belt_tamp
} // namespace drake