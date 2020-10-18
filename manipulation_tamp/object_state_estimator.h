#include <string>
#include <iostream>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcmt_combined_object_state.hpp"

#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace manipulation_tamp {
namespace manipulation_station {

using multibody::BodyIndex;
using multibody::SpatialVelocity;

template <typename T>
class ObjectStateEstimator : public systems::LeafSystem<T> {
public:
explicit ObjectStateEstimator(multibody::MultibodyPlant<T>* plant,
    std::vector<BodyIndex>* object_ids);

// calc callback function
void OutputObjectState(
    const systems::Context<T>& context, lcmt_combined_object_state* output) const;

// alloc callback function
lcmt_combined_object_state MakeOutputMsg() const;

const systems::InputPort<T>& get_input_port_body_poses() const;
const systems::InputPort<T>& get_input_port_body_velocities() const;
const systems::OutputPort<T>& get_output_port_msg() const;

private:
const multibody::MultibodyPlant<T>* plant_;
std::vector<BodyIndex> object_ids_;
int output_port_index_msg_{0};
int input_port_body_poses_{0};
int input_port_body_velocities_{0};
};

} // namespace manipulation_station
} // namespace manipulation_tamp
} // namespace drake
