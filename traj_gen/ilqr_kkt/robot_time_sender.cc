#include "drake/traj_gen/ilqr_kkt/robot_time_sender.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

RobotTimeSender::RobotTimeSender()
{
  this->DeclareAbstractOutputPort(
      "lcmt_robot_time", &RobotTimeSender::CalcOutput);
}

const systems::OutputPort<double>& RobotTimeSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void RobotTimeSender::CalcOutput(
    const systems::Context<double>& context, lcmt_robot_time* output) const {

  lcmt_robot_time& robot_time = *output;
  robot_time.utime = context.get_time() * 1e6;
}

} // namespace drake 
} // namespace traj_gen 
} // namespace kuka_iiwa_arm