#include <iostream>
#include <memory>

#include "gflags/gflags.h"
#include "lcm/lcm-cpp.hpp"

#include "drake/common/trajectories/piecewise_polynomial.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/lcmt_generic_string_msg.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"


namespace drake {
namespace manipulation_tamp {
namespace manipulation_station {

using trajectories::PiecewisePolynomial;

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kExecutionStatusChannel = "EXECUTION_STATUS";
const char* const kLcmSchunkWsgStatusChannel = "SCHUNK_WSG_STATUS";
const char* const kLcmSchunkWsgCommandChannel = "SCHUNK_WSG_COMMAND";

const int kNumIiwaJoints = 7;
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
    std::unique_ptr<PiecewisePolynomial<double>> plan_;
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

        if (has_active_plan_) {
            if (plan_number_ != cur_plan_number_) {
                std::cout<<"Starting new plan.\n";
                start_utime_ = iiwa_status_.utime;
                cur_plan_number_ = plan_number_;
                cur_traj_idx_ = 0;
            }

            cur_traj_time_sec_ = (iiwa_status_.utime - start_utime_)/1e6;
            cur_traj_time_sec_ *= trajTimeScale;

            if (cur_traj_time_sec_ >= 
                    manip_traj_.times_sec[manip_traj_.n_time_steps-1]) {
                std::cout<<"Current plan completed. Waiting for new plan\n";

                has_active_plan_ = false;
                plan_.reset();

                lcmt_generic_string_msg plan_status;
                plan_status.msg = "Finished";
                lcm_.publish(kExecutionStatusChannel, &plan_status);
                return;
            }

            while (cur_traj_time_sec_ > manip_traj_.times_sec[cur_traj_idx_]) {
                cur_traj_idx_++;
            }

            const auto desired_next = plan_->value(cur_traj_time_sec_);

            // if new traj time step reached, publish new command
            iiwa_command_.utime = iiwa_status_.utime;
            iiwa_command_.num_torques = 0;
            iiwa_command_.num_joints = kNumIiwaJoints;
            iiwa_command_.joint_position.resize(iiwa_command_.num_joints);
            iiwa_command_.joint_torque.resize(iiwa_command_.num_torques);

            for (int i=0; i<kNumIiwaJoints; i++) {
                iiwa_command_.joint_position[i] = desired_next(i);
            }

            lcm_.publish(kLcmCommandChannel, &iiwa_command_);

            wsg_command_.utime = wsg_status_.utime;
            wsg_command_.target_position_mm = manip_traj_.gripper_width[cur_traj_idx_];
            wsg_command_.force = manip_traj_.gripper_force[cur_traj_idx_];

            lcm_.publish(kLcmSchunkWsgCommandChannel, &wsg_command_);
        } else {
            iiwa_command_.utime = iiwa_status_.utime;
            iiwa_command_.num_torques = 0;
            iiwa_command_.num_joints = kNumIiwaJoints;
            iiwa_command_.joint_position = iiwa_status_.joint_position_commanded;
            lcm_.publish(kLcmCommandChannel, &iiwa_command_);

            wsg_command_.utime = wsg_status_.utime;
            wsg_command_.target_position_mm = wsg_status_.actual_position_mm;
            wsg_command_.force = wsg_status_.actual_force;
            lcm_.publish(kLcmSchunkWsgCommandChannel, &wsg_command_);
        }
    }

    void HandleIiwaTraj(const lcm::ReceiveBuffer*, const std::string&,
                        const lcmt_manipulator_traj* traj) {
        std::cout<<"Received new plan.\n";

        if (iiwa_status_.utime == -1) {
            std::cout<<"Discarding plan, no status message received yet\n";
            return;
        } else if (traj->n_time_steps < 2) {
            std::cout << "Discarding plan, Not enough knot points." << std::endl;
            return;
        }

        manip_traj_ = *traj;

        std::vector<Eigen::MatrixXd> knots(manip_traj_.n_time_steps,
                                           Eigen::MatrixXd::Zero(kNumIiwaJoints, 1));

        for (int i = 0; i < manip_traj_.n_time_steps; i++) {
            for (int j = 0; j < kNumIiwaJoints; j++) {
                if (i == 0) {
                    // always start moving from the position which we are 
                    // currently commanding
                    DRAKE_DEMAND(iiwa_status_.utime != -1);
                    knots[0](j, 0) = iiwa_status_.joint_position_commanded[j];
                } else {
                    knots[i](j, 0) = manip_traj_.states[i][j];
                }
            }
        }

        plan_.reset(new PiecewisePolynomial<double>(
            PiecewisePolynomial<double>::FirstOrderHold(
                manip_traj_.times_sec, knots)));
        std::cout<< "Plan Interpolation Completed"<<"\n";
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
