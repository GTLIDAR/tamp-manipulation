#include <iostream>
#include <memory>

#include "gflags/gflags.h"
#include "lcm/lcm-cpp.hpp"

#include "drake/common/find_resource.h"
#include "drake/multibody/tree/multibody_tree_system.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/lcmt_generic_string_msg.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

using drake::multibody::internal::MultibodyTree;

namespace drake {
namespace manipulation_tamp {
namespace manipulation_station {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kExecutionStatusChannel = "EXECUTION_STATUS";
const char* const kLcmSchunkWsgStatusChannel = "SCHUNK_WSG_STATUS";
const char* const kLcmSchunkWsgCommandChannel = "SCHUNK_WSG_COMMAND";

const int kNumIiwaJoints = 7;
const int kIiwaTorqueStartIdx = 0; // in case torque vector is expanded in future
const int trajTimeScale = 1;

class RobotPlanRunner {
    public:
    RobotPlanRunner() {
        lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleIiwaStatus, this);
        lcm_.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandleIiwaTraj, this);
        lcm_.subscribe(kLcmSchunkWsgStatusChannel, &RobotPlanRunner::HandleWsgStatus, this);
        iiwa_status_.utime = -1;
        plan_number_ = 0;
        has_active_plan_ = false;
        cur_plan_number_ = plan_number_;
        cur_traj_idx_ = 0;
        cur_traj_time_sec_ = 0;
        start_utime_ = 0;
        iiwa_command_.num_joints = kNumIiwaJoints;
    }

    void Run() {

        while (true) {
            while (lcm_.handleTimeout(10)==0 || iiwa_status_.utime==-1) {}
        }
    }

    private:
    lcm::LCM lcm_;
    int plan_number_;
    lcmt_iiwa_status iiwa_status_;
    lcmt_iiwa_command iiwa_command_;
    lcmt_manipulator_traj manip_traj_;
    bool has_active_plan_;
    int cur_plan_number_;
    int cur_traj_idx_;
    double cur_traj_time_sec_;
    int64_t start_utime_;
    lcmt_schunk_wsg_status wsg_status_;
    lcmt_schunk_wsg_command wsg_command_;

    void HandleWsgStatus(const lcm::ReceiveBuffer*, const std::string&,
                          const lcmt_schunk_wsg_status* status) {
        wsg_status_ = *status;
    }

    void HandleIiwaStatus(const lcm::ReceiveBuffer*, const std::string&,
                          const lcmt_iiwa_status* status) {
        iiwa_status_ = *status;

        // std::cout << "Torque: " << std::endl;
        // for (int i=0; i<status->num_joints; i++)
        //     std::cout << status->joint_torque_measured[i] << " ";
        // std::cout << std::endl;

        // new command is published here for easier clock sync
        if (has_active_plan_) {
            if (plan_number_ != cur_plan_number_) {
                std::cout<<"Starting new plan.\n";
                start_utime_ = iiwa_status_.utime;
                cur_plan_number_ = plan_number_;
                cur_traj_idx_ = 0;
            }

            cur_traj_time_sec_ = (iiwa_status_.utime - start_utime_)/1e6;
            cur_traj_time_sec_ *= trajTimeScale;
            if (cur_traj_idx_ % 10 == 0) {
                std::cout<<"Total length: "<<manip_traj_.times_sec.size()<<"\n";
                std::cout<<"Cur Idx: "<<cur_traj_idx_<<"\n";
                std::cout<< "Plan Runner Time: "<<cur_traj_time_sec_<<"\n";
                std::cout<< "Plan Time: "<<manip_traj_.times_sec[cur_traj_idx_]<<"\n\n";
            }
            // increment cur_traj_idx until appropriate command is found
            while (cur_traj_time_sec_ > manip_traj_.times_sec[cur_traj_idx_]) {
                cur_traj_idx_++;
                // check if current traj is finished
                if (cur_traj_idx_ >= manip_traj_.n_time_steps) {
                    std::cout<<"Current plan completed. Waiting for new plan\n";

                    has_active_plan_ = false;

                    lcmt_generic_string_msg plan_status;
                    plan_status.msg = "Finished";
                    lcm_.publish(kExecutionStatusChannel, &plan_status);
                    return;
                }
            }

            // if new traj time step reached, publish new command
            iiwa_command_.utime = iiwa_status_.utime;
            // iiwa_command_.num_torques = manip_traj_.dim_torques;
            iiwa_command_.num_torques = 0;
            iiwa_command_.joint_position.resize(iiwa_command_.num_joints);
            iiwa_command_.joint_torque.resize(iiwa_command_.num_torques);

            // if (manip_traj_.dim_torques) {
            //     iiwa_command_.joint_position = iiwa_status_.joint_position_measured;
            //     for (int i=0; i<kNumIiwaJoints; i++) {
            //         iiwa_command_.joint_torque[i] =
            //             manip_traj_.torques[cur_traj_idx_][i+kIiwaTorqueStartIdx];
            //     }
            // } else {
            for (int i=0; i<kNumIiwaJoints; i++) {
                iiwa_command_.joint_position[i] =
                    manip_traj_.states[cur_traj_idx_][i+kIiwaTorqueStartIdx];
            }
            // }

            lcm_.publish(kLcmCommandChannel, &iiwa_command_);

            wsg_command_.utime = wsg_status_.utime;
            wsg_command_.target_position_mm = manip_traj_.gripper_width[cur_traj_idx_];
            wsg_command_.force = manip_traj_.gripper_force[cur_traj_idx_];

            lcm_.publish(kLcmSchunkWsgCommandChannel, &wsg_command_);


            cur_traj_idx_++;

        }
    }

    void HandleIiwaTraj(const lcm::ReceiveBuffer*, const std::string&,
                        const lcmt_manipulator_traj* plan) {
        std::cout<<"Received new plan.\n";

        if (iiwa_status_.utime == -1) {
            std::cout<<"Discarding plan, no status message received yet\n";
            return;
        }

        manip_traj_ = *plan;
        has_active_plan_ = true;
        plan_number_++;
    }
};

} // namespace manipulation station
} // namespace examples
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::manipulation_tamp::manipulation_station::RobotPlanRunner runner;
    runner.Run();
    return 0;
}
