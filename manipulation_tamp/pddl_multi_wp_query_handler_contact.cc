#include <iostream>
#include <vector>
#include <string>

#include "gflags/gflags.h"
#include "lcm-cpp.hpp"

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/lcmt_multi_wp_manip_query.hpp"
#include "drake/lcmt_manipulator_traj.hpp"
#include "drake/traj_gen/ddp_runner.h"
#include "drake/traj_gen/admm_runner.h"
#include "drake/traj_gen/ilqr_kkt/ddp_runner_contact.h"
#include "drake/traj_gen/ilqr_kkt/admm_runner_contact.h"

DEFINE_double(gripper_open_width, 100, "Width gripper opens to in mm");
DEFINE_double(gripper_close_width, 10, "Width gripper closes to in mm");
DEFINE_double(gripper_push_width, 50, "Width gripper closes to in mm");

DEFINE_string(
    KukaIiwaUrdf,
    "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf",
    "file name of iiwa7 urdf"
);
DEFINE_string(
    query_channel,
    "TREE_SEARCH_QUERY",
    "channel for tree search to send its path plan query"
);
DEFINE_string(
    result_channel,
    "TREE_SEARCH_QUERY_RESULTS",
    "channel for low level to send back query results"
);
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

using drake::manipulation::planner::ConstraintRelaxingIk;
using drake::traj_gen::kuka_iiwa_arm::ADMMRunner;
using drake::traj_gen::kuka_iiwa_arm::DDPRunner;
using drake::traj_gen::kuka_iiwa_arm::DDP_KKTRunner;
using drake::traj_gen::kuka_iiwa_arm::ADMM_KKTRunner;

namespace real_lcm = lcm;

namespace drake {
namespace manipulation_tamp {

class PDDLQueryHandler {

public:
PDDLQueryHandler() {
    // Q_POST_THROW_ << -3.40486e-12, 0.743339, 8.16463e-12,  -0.5, 8.20197e-12,
    //     -0.6 ,-1.01735e-11;
    model_path_ = FindResourceOrThrow(FLAGS_KukaIiwaUrdf);
    lcm_.subscribe(FLAGS_query_channel, &PDDLQueryHandler::HandleQuery, this);
}

void Run() {
    while (lcm_.handle()>=0);
}

private:
std::string model_path_;
real_lcm::LCM lcm_;
lcmt_multi_wp_manip_query current_query_;
lcmt_manipulator_traj traj_;
lcmt_manipulator_traj total_traj_;

void HandleQuery(
    const real_lcm::ReceiveBuffer*,
    const std::string&,
    const lcmt_multi_wp_manip_query* query
) {
    current_query_ = *query;
    if (query->wait_time < 0) {
        std::cout<<"Negative wait time, this usually means push is not feasible\n";
        traj_ = GetInfCost();
        lcm_.publish(FLAGS_result_channel, &traj_);
        return;
    }

    // initialize
    VectorXd iiwa_q = VectorXd::Zero(query->dim_q);
    for (int i = 0; i < query->dim_q; i++) {
        iiwa_q[i] = query->prev_q[i];
    }

    std::vector<VectorXd> q_sol;
    q_sol.push_back(iiwa_q);
    bool ik_feasible;
    if (query->bypass_ik) {
        std::cout<<"Bypassing IK...\n";
        ik_feasible = true;
        
        VectorXd q_goal = VectorXd::Zero(query->dim_q);
        for (int n_wp = 0; n_wp < query->n_wp; n_wp++) {
            for (int i = 0; i < query->dim_q; i++) {
                q_goal[i] = query->q_goal[n_wp][i];
            }
            q_sol.push_back(q_goal);
        }

    } else {
        ConstraintRelaxingIk ik(
            model_path_,
            FLAGS_ee_name
        );

        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
        std::vector<VectorXd> q_sol_wp;
        
        for (int i = 0; i < query->n_wp; i++) {
            ConstraintRelaxingIk::IkCartesianWaypoint wp;
            const Eigen::Vector3d xyz(
                query->desired_ee[i][0],
                query->desired_ee[i][1],
                query->desired_ee[i][2]
            );
            const math::RollPitchYaw<double> rpy(
                query->desired_ee[i][3],
                query->desired_ee[i][4],
                query->desired_ee[i][5]
            );

            wp.pose.set_translation(xyz);
            wp.pose.set_rotation(rpy);
            wp.constrain_orientation = query->constrain_orientation[i];

            wp_vec.push_back(wp);
            ik_feasible = ik.PlanSequentialTrajectory(wp_vec, q_sol.back(), &q_sol_wp);
            
            if (!ik_feasible) {
                std::cout<<"infeasible at wp "<<i<<"\n";
                break;
            } else {
                std::cout<<"feasible at wp "<<i<<"\n";
            }

            q_sol.push_back(q_sol_wp.back());
            wp_vec.clear();
            q_sol_wp.clear();
        }

        
    }

    // lcmt_manipulator_traj traj;
    if (!ik_feasible) {
        std::cout<<"IK infeasuble\n";
        traj_ = GetInfCost();
        lcm_.publish(FLAGS_result_channel, &traj_);
        return;
    }

    std::cout<<"\n";
    // lcmt_manipulator_traj total_traj_;
    total_traj_.dim_states = query->dim_q;
    total_traj_.dim_torques = 0;
    total_traj_.n_time_steps = 0;

    VectorXd qo_init;
    VectorXd qo_final;

    VectorXd q_init;
    q_init = iiwa_q;

    for (size_t i = 0; i < q_sol.size()-1; i++) {
        if (query->option.compare("ddp")==0) {
            if (query->use_object) {
                qo_init = VectorXd::Zero(13);
                qo_final = VectorXd::Zero(13);
                if (i==0) {
                    for (int j = 0; j < 13; j++) {
                        qo_init[j] = query->q_init_object[j];
                    }
                } else {
                    for (int j = 0; j < 13; j++) {
                        qo_init[j] = query->q_desired_object[i-1][j];
                    }
                }

                for (int j = 0; j < 13; j++) {
                    qo_final[j] = query->q_desired_object[i][j];
                }
                traj_ = GetDDPContactRes(
                    q_init, q_sol[i+1], qo_init, qo_final, query->time_horizon[i], query->time_step, query->name
                );
            } else {
                traj_ = 
                    GetDDPRes(q_init, q_sol[i+1], query->time_horizon[i], query->time_step);
            }

        } else if (query->option.compare("admm")==0) {
            if (query->use_object) {
                qo_init = VectorXd::Zero(13);
                qo_final = VectorXd::Zero(13);
                if (i==0) {
                    for (int j = 0; j < 13; j++) {
                        qo_init[j] = query->q_init_object[j];
                    }
                } else {
                    for (int j = 0; j < 13; j++) {
                        qo_init[j] = query->q_desired_object[i-1][j];
                    }
                }

                for (int j = 0; j < 13; j++) {
                    qo_final[j] = query->q_desired_object[i][j];
                }
                traj_ = GetADMMContactRes(
                    q_init, q_sol[i+1], qo_init, qo_final, query->time_horizon[i], query->time_step, query->name
                );
            } else {
                traj_ = 
                    GetADMMRes(q_init, q_sol[i+1], query->time_horizon[i], query->time_step, query->name);
            }
        } else {
            std::cout<<"Invalid Traj Op Option. Only support ddp or admm!\n";
        }

        for (int j = 0; j < traj_.dim_states; j++) {
            q_init[j] = traj_.states[traj_.n_time_steps-1][j];
        }

        std::vector<double> widths;
        std::vector<double> forces;
        forces.assign(traj_.n_time_steps, query->gripper_force);
        if (query->name.find("wait")==0) {
            widths.assign(traj_.n_time_steps, query->prev_gripper_width);
        } else if (query->name.find("release")==0) {
            widths.assign(traj_.n_time_steps, FLAGS_gripper_open_width);
        } else if (query->name.find("grasp")==0) {
            widths.assign(traj_.n_time_steps, FLAGS_gripper_close_width);
        } else if (query->name.find("push")==0) {
            widths.assign(traj_.n_time_steps, FLAGS_gripper_push_width);
        } else if (query->name.find("throw")==0) {
            if (i < q_sol.size()-2) {
                widths.assign(traj_.n_time_steps, FLAGS_gripper_close_width);
            } else {
                for (int j = 0; j < traj_.n_time_steps; j++) {
                    if (j < traj_.n_time_steps/5.) {
                        widths.push_back(FLAGS_gripper_close_width);
                    } else {
                        widths.push_back(FLAGS_gripper_open_width);
                    }
                }
            }
        } else if (query->name.find("move-to-object")==0) {
            widths.assign(traj_.n_time_steps, FLAGS_gripper_open_width);
        } else if (query->name.find("move")==0) {
            widths.assign(traj_.n_time_steps, query->prev_gripper_width);
        } else {
            std::cout << "This shouldn't happen, Assigning gripper to previous state\n";
            widths.assign(traj_.n_time_steps, query->prev_gripper_width);
        }
        traj_.gripper_force = forces;
        traj_.gripper_width = widths;

        AppendTrajectory(total_traj_, traj_);

        if (isnan(traj_.cost)) {
            std::cout<<"This trajectory returns NaN cost!\n";
            std::cout<<"IK Results"<<"\n";
            std::cout<<"q_init ";
            for (int j = 0; j < kNumJoints; j++) {
                std::cout<<q_sol[i][j]<<" ";
            }
            std::cout<<"\nq_goal";
            for (int j = 0; j < kNumJoints; j++) {
                std::cout<<q_sol[i+1][j]<<" ";
            }
            std::cout<<"\n";
            break;
        }
        ClearTrajectory(traj_);
    }

    // add wait time in traj
    if (query->wait_time) {
        int wait_time_steps = query->wait_time / query->time_step;
        total_traj_.n_time_steps += wait_time_steps;
        double time_step = query->time_step;
        for (int ts = 0; ts < wait_time_steps; ts++) {
            total_traj_.times_sec.push_back(total_traj_.times_sec.back()+time_step);
            total_traj_.states.push_back(total_traj_.states.back());
            total_traj_.torques.push_back(total_traj_.torques.back());
            total_traj_.gripper_width.push_back(total_traj_.gripper_width.back());
            total_traj_.gripper_force.push_back(total_traj_.gripper_force.back());
        }
        std::cout<<"wait traj added.\n";
    }

    lcm_.publish(FLAGS_result_channel, &total_traj_);
    for (size_t i = 0; i < q_sol.size()-1; i++) {
        std::cout<<"IK Results"<<"\n";
        std::cout<<"q_init ";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i][j]<<" ";
        }
        std::cout<<"\nq_goal";
        for (int j = 0; j < kNumJoints; j++) {
            std::cout<<q_sol[i+1][j]<<" ";
        }
        std::cout<<"\n";
    }
    std::cout << "--------"<<query->name<<" Trajectory Published to LCM! --------" << endl;
    ClearTrajectory(total_traj_);
}

lcmt_manipulator_traj GetInfCost() {
    std::cout<<"IK FAILED, publishing Inf cost\n";
    auto msg = std::make_unique<lcmt_manipulator_traj>();
    msg->cost = std::numeric_limits<double>::infinity();
    msg->n_time_steps = 0;
    msg->dim_states = 0;
    msg->dim_torques = 0;

    return *msg;
}

lcmt_manipulator_traj GetDDPRes(VectorXd q_init, VectorXd q_goal,
    double time_horizon, double time_step) {
    std::cout<<"IK Successful, sent to ddp\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";

    DDPRunner runner;
    return runner.RunDDP(qv_init, qv_goal, time_horizon, time_step);
}

lcmt_manipulator_traj GetDDPContactRes(VectorXd q_init, VectorXd q_goal, 
    VectorXd q_obj_init, VectorXd q_obj_goal, 
    double time_horizon, double time_step, string action_name) {
    std::cout<<"IK Successful, sent to ddp\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(fullstateSize);
    VectorXd::Map(&qv_init[0], q_obj_init.size()) = q_obj_init;
    VectorXd::Map(&qv_init[q_obj_init.size()], q_init.size()) = q_init;
    std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(fullstateSize);
    VectorXd::Map(&qv_goal[0], q_obj_goal.size()) = q_obj_goal;
    VectorXd::Map(&qv_goal[q_obj_goal.size()], q_goal.size()) = q_goal;
    std::cout<<"qv_goal:\n"<<qv_goal<<"\n";

    DDP_KKTRunner runner;
    return runner.RunDDP_KKT(qv_init, qv_goal, time_horizon, time_step, action_name);
}

lcmt_manipulator_traj GetADMMRes(VectorXd q_init, VectorXd q_goal,
    double time_horizon, double time_step, string action_name) {
    std::cout<<"IK Successful, sent to admm\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_init[0], q_init.size()) = q_init;
    // std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(stateSize);
    VectorXd::Map(&qv_goal[0], q_goal.size()) = q_goal;
    // std::cout<<"qv_goal:\n"<<qv_goal<<"\n";
    
    ADMMRunner runner;
    return runner.RunADMM(qv_init, qv_goal, time_horizon, time_step, action_name);
}

lcmt_manipulator_traj GetADMMContactRes(VectorXd q_init, VectorXd q_goal,
    VectorXd q_obj_init, VectorXd q_obj_goal, 
    double time_horizon, double time_step, string action_name) {
    std::cout<<"IK Successful, sent to admm\n";

    VectorXd qv_init;
    qv_init = Eigen::VectorXd::Zero(fullstateSize);
    VectorXd::Map(&qv_init[0], q_obj_init.size()) = q_obj_init;
    VectorXd::Map(&qv_init[q_obj_init.size()], q_init.size()) = q_init;
    std::cout<<"qv_init:\n"<<qv_init<<"\n";

    VectorXd qv_goal;
    qv_goal = VectorXd::Zero(fullstateSize);
    VectorXd::Map(&qv_goal[0], q_obj_goal.size()) = q_obj_goal;
    VectorXd::Map(&qv_goal[q_obj_goal.size()], q_goal.size()) = q_goal;
    std::cout<<"qv_goal:\n"<<qv_goal<<"\n";
    
    ADMM_KKTRunner runner;
    return runner.RunADMM_KKT(qv_init, qv_goal, time_horizon, time_step, action_name);
}

void AppendTrajectory(lcmt_manipulator_traj &dest, lcmt_manipulator_traj &src) {
    DRAKE_DEMAND(dest.dim_states==src.dim_states);
    DRAKE_DEMAND(dest.dim_torques==src.dim_torques);

    dest.n_time_steps += src.n_time_steps;
    dest.cost += src.cost;
    std::vector<double> times_sec = src.times_sec;
    if (dest.times_sec.size()) {
        for (size_t i = 0; i < times_sec.size(); i++) {
            times_sec[i] += dest.times_sec.back() + current_query_.time_step;
        }
    }

    dest.times_sec.insert(
        dest.times_sec.end(), times_sec.begin(), times_sec.end());
    dest.states.insert(
        dest.states.end(), src.states.begin(), src.states.end()); 
    dest.torques.insert(
        dest.torques.end(), src.torques.begin(), src.torques.end());
    dest.gripper_width.insert(
        dest.gripper_width.end(), src.gripper_width.begin(), src.gripper_width.end());
    dest.gripper_force.insert(
        dest.gripper_force.end(), src.gripper_force.begin(), src.gripper_force.end());
}

void ClearTrajectory(lcmt_manipulator_traj &traj) {
    traj.n_time_steps = 0;
    traj.dim_torques = 0;
    traj.dim_states = 0;
    traj.times_sec.clear();
    traj.states.clear();
    traj.torques.clear();
    traj.gripper_width.clear();
    traj.gripper_force.clear();
    traj.cost = 0;
}

};
} // namespace conveyor_belt_iiwa
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::manipulation_tamp::PDDLQueryHandler handler;
    handler.Run();
    return 0;
}
