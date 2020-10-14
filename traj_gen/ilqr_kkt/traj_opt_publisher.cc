#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_manipulator_traj.hpp"
#include "gflags/gflags.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"

#include "drake/lcmt_generic_string_msg.hpp"

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/ilqr_kkt/ilqr-kkt_solver.h"
#include "drake/traj_gen/ilqr_kkt/kuka_arm_contact.h"

#include "drake/lcmt_motion_plan_query.hpp"

DEFINE_double(gripper_open_width, 100, "Width gripper opens to in mm");
DEFINE_double(gripper_close_width, 10, "Width gripper closes to in mm");
DEFINE_double(gripper_force, 50, "force for gripper");
DEFINE_double(table_width, 0.7112, "Width of table supporting kuka arm");
DEFINE_double(belt_width, 0.4, "Width of conveyor belt");
DEFINE_double(default_iiwa_x, 0.2, "X position of iiwa base");
DEFINE_double(kConveyorBeltTopZInWorld, 0.736 + 0.02 / 2, "height of belt");
DEFINE_string(plan_channel, "COMMITTED_ROBOT_PLAN", "Plan channels for plan");
DEFINE_string(
    KukaIiwaUrdf,
    "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf",
    "file name of iiwa7 urdf"
);
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

using namespace std;
using namespace Eigen;
using lcm::LCM;

#define useILQRSolver 1
#define useUDPSolver 0

/* DDP trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmObjectStatusChannel = "OBJECT_STATUS";
const char* const kLcmSchunkStatusChannel = "WSG_STATUS";
const char* const kLcmTimeChannel = "IIWA_TIME";

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using multibody::ModelInstanceIndex;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyForces;
using multibody::BodyIndex;
using manipulation::planner::ConstraintRelaxingIk;

class TrajOptPublisher
{
private:
    LCM lcm_;
    lcmt_robot_time robot_time_;
    bool plan_finished_;
    unsigned int step_;
    fullstateVecTab_t joint_state_traj;
    commandVecTab_t torque_traj;
    fullstateVecTab_t joint_state_traj_interp;
    commandVecTab_t torque_traj_interp;

public:
    explicit TrajOptPublisher()
    {
        lcm_.subscribe(kLcmTimeChannel,
                        &TrajOptPublisher::HandleRobotTime, this);
    }

    void Run_test(fullstateVec_t xinit, fullstateVec_t xgoal, double time_horizon, double time_step, double realtime_rate, string action_name){
        struct timeval tbegin,tend;
        double texec = 0.0;
        double dt = time_step;
        unsigned int N = int(time_horizon/time_step);
        double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
        double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
        unsigned int iterMax = 30; //100;    

        ILQR_KKTSolver::traj lastTraj;
        //=============================================
        // Build wholebody and pass over to kukaArm
        std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
        std::string schunkPath = 
          FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf");
        std::string connectorPath = 
          FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");
        const std::string box_sdf_path0 = "drake/conveyor_belt_tamp/models/boxes/redblock.urdf";
        // const std::string box_sdf_path0 = "drake/conveyor_belt_tamp/models/boxes/large_red_box.urdf";

        std::string urdf_;
        auto plant_ = multibody::MultibodyPlant<double>(0.0);
        multibody::Parser parser(&plant_);
        
        const ModelInstanceIndex iiwa_model = 
          parser.AddModelFromFile(kIiwaUrdf, "iiwa");
        const auto& iiwa_base_frame = plant_.GetFrameByName("iiwa_link_0", iiwa_model);
        RigidTransformd X_WI(Eigen::Vector3d(0, 0, 0));
        plant_.WeldFrames(plant_.world_frame(), iiwa_base_frame, X_WI);

        const ModelInstanceIndex conn_model = 
          parser.AddModelFromFile(connectorPath, "connector");
        const auto& iiwa_ee_frame = plant_.GetFrameByName("iiwa_frame_ee", iiwa_model);
        const auto& conn_frame = plant_.GetFrameByName("connector_link", conn_model);
        RigidTransformd X_EC(Eigen::Vector3d(0, 0, 0));
        plant_.WeldFrames(iiwa_ee_frame, conn_frame, X_EC);

        const ModelInstanceIndex wsg_model = 
          parser.AddModelFromFile(schunkPath, "wsg");
        const auto& wsg_frame = plant_.GetFrameByName("body", wsg_model);
        RigidTransformd X_EG(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                        Vector3d(0, 0, 0.053));
        plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

        // const ModelInstanceIndex object_model =
        parser.AddModelFromFile(FindResourceOrThrow(box_sdf_path0), "object");

        plant_.Finalize();

        auto context_ptr = plant_.CreateDefaultContext();
        auto context = context_ptr.get();

        VectorXd q_v_iiwa(14);
        q_v_iiwa.setZero();
        q_v_iiwa.head(7) = xinit;
        plant_.SetPositionsAndVelocities(context, iiwa_model, q_v_iiwa);

        MatrixXd M_(plant_.num_velocities(), plant_.num_velocities());
        plant_.CalcMassMatrix(*context, &M_);

        VectorXd gtau_wb = plant_.CalcGravityGeneralizedForces(*context);

        // cout << "bias total" << endl << gtau_wb << endl;
        commandVecTab_t u_0;
        u_0.resize(N);
        for(unsigned i=0;i<N;i++){
        //   u_0[i] = -gtau_wb.middleRows<kNumJoints>(6);
          // cout << "u_0: " << u_0[i].transpose() << endl;
          u_0[i].setZero();
          // u_0[i] << 10, 10, 10, 10, 10, 10, 10;
        }
        //======================================
        KukaArm_Contact KukaArmModel(dt, N, xgoal, &plant_, action_name);
        CostFunctionKukaArm_Contact costKukaArm(N, action_name);
        ILQR_KKTSolver testSolverKukaArm(KukaArmModel,costKukaArm,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverKukaArm.firstInitSolver(xinit, xgoal, u_0, N, dt, iterMax, tolFun, tolGrad);     

        // run one or multiple times and then average
        unsigned int Num_run = 1;
        gettimeofday(&tbegin,NULL);
        for(unsigned int i=0;i<Num_run;i++) {testSolverKukaArm.solveTrajectory();}
        if(Num_run == 0) {testSolverKukaArm.initializeTraj();}
        gettimeofday(&tend,NULL);

        lastTraj = testSolverKukaArm.getLastSolvedTrajectory();
        #if useUDPSolver
        finalTimeProfile = KukaArmModel.getFinalTimeProfile();
        #endif
        joint_state_traj.resize(N+1);
        joint_state_traj_interp.resize(N*InterpolationScale+1);
        for(unsigned int i=0;i<=N;i++){
        joint_state_traj[i] = lastTraj.xList[i];
        }
        torque_traj = lastTraj.uList;

        //linear interpolation to 1ms
        for(unsigned int i=0;i<fullstateSize;i++){
        for(unsigned int j=0;j<N*InterpolationScale;j++){
        unsigned int index = j/10;
        joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
        }
        joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
        }

        texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
        texec /= Num_run;

        cout << endl;
        cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
        cout << "Final cost: " << lastTraj.finalCost << endl;
        cout << "Final gradient: " << lastTraj.finalGrad << endl;
        cout << "Final lambda: " << lastTraj.finalLambda << endl;
        cout << "Execution time by time step (second): " << texec/N << endl;
        cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
        cout << "Total execution time of the solver (second): " << texec << endl;
        cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
        cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;

        cout << "xgoal: " << xgoal.transpose() << endl;
        cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList[N].transpose() << endl;
        cout << "lastTraj.uList[" << N-1 << "]:" << lastTraj.uList[N-1].transpose() << endl;

        cout << "lastTraj.xList[0]:" << lastTraj.xList[0].transpose() << endl;
        cout << "lastTraj.uList[0]:" << lastTraj.uList[0].transpose() << endl;

        // auto context_ptr = plant_.CreateDefaultContext();
        // auto context = context_ptr.get();
        // for(unsigned int i=N-50;i<=N;i++){
        //   // Do the forward kinamtics for the EE to check the performance
        //   auto rpy = math::RollPitchYawd(Eigen::Vector3d(0, 0, 0));
        //   auto xyz = Eigen::Vector3d(0, 0, 0);
        //   math::RigidTransform<double> X_WO(math::RotationMatrix<double>(rpy), xyz);
        //   plant_.SetFreeBodyPoseInWorldFrame(context, plant_.GetBodyByName("base_link", object_model), X_WO);
        //   plant_.SetPositions(context, iiwa_model, lastTraj.xList[i].topRows(7));
        //   plant_.SetVelocities(context, iiwa_model, lastTraj.xList[i].bottomRows(7));
        //   const auto& X_WB_all = plant_.get_body_poses_output_port().Eval<std::vector<math::RigidTransform<double>>>(*context);
        //   const BodyIndex ee_body_index = plant_.GetBodyByName("iiwa_link_ee_kuka", iiwa_model).index();
        //   const math::RigidTransform<double>& X_Wee = X_WB_all[ee_body_index];
        //   cout << "ee[" << i << "]:" << X_Wee.translation().transpose() << endl;
        // }
        // saving data file
        for(unsigned int i=0;i<N;i++){
        saveVector(joint_state_traj[i], "joint_trajectory");
        saveVector(torque_traj[i], "joint_torque_command");
        }
        saveVector(lastTraj.xList[N], "joint_trajectory");

        for(unsigned int i=0;i<=N*InterpolationScale;i++){
        saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated");
        }

        cout << "-------- DDP Trajectory Generation Finished! --------" << endl;

        ////////////////////////// Was used for publishing to kuka_arm_planner ////////////////////////
        // need this for dynamic memory allocation (push_back)
        // auto ptr = std::make_unique<lcmt_manipulator_traj>();
        
        // ptr->dim_torques = 0;//kNumJoints;
        // ptr->dim_states = kNumJoints; //disregard joint velocity
        // ptr->n_time_steps = N*InterpolationScale; 
        // ptr->cost = lastTraj.finalCost;

        // for (int32_t i=0; i < ptr->n_time_steps; ++i) {
        // // need new, cuz dynamic allocation or pointer
        // ptr->times_sec.push_back(static_cast<double>(time_step*i/InterpolationScale));
        // // cout << time_step*i/InterpolationScale << endl;

        // // cout << ptr->times_sec[i] << endl;

        // auto ptr2 = std::make_unique<std::vector<double>>();
        // auto ptr2_st = std::make_unique<std::vector<double>>();

        // for (int32_t j=0; j < ptr->dim_states; ++j) { 
        //     //  ptr2->push_back(lastTraj.uList[i][j]);
        //     // ptr2->push_back(gtau[j]);

        //     ptr2->push_back(0);
        //     ptr2_st->push_back(joint_state_traj_interp[i][j]);
        // }
        // ptr->torques.push_back(*ptr2);
        // ptr->states.push_back(*ptr2_st);
        // }

        // lcmt_manipulator_traj traj = *ptr;
        // std::vector<double> widths;
        // std::vector<double> forces;
        // forces.assign(traj.n_time_steps, FLAGS_gripper_force);
        // widths.assign(traj.n_time_steps, FLAGS_gripper_close_width);

        // traj.gripper_force = forces;
        // traj.gripper_width = widths;

        // std::cout<<"Press any key to continue...\n";
        // while (std::getc(stdin)==EOF) {}

        // lcm_.publish(FLAGS_plan_channel, &traj);
        // std::cout<<"Trajectory Published\n";
        // return *ptr;

        //////////////////////////////////////////////////////////////////////////////////////////
        lcmt_iiwa_status iiwa_state;
        lcmt_schunk_wsg_status wsg_status;
        lcmt_object_status object_state;
        iiwa_state.num_joints = kIiwaArmNumJoints;
        object_state.num_joints = 7;
        DRAKE_ASSERT(plant_.num_positions(iiwa_model) == kIiwaArmNumJoints);
        iiwa_state.joint_position_measured.resize(kIiwaArmNumJoints, 0.);   
        iiwa_state.joint_velocity_estimated.resize(kIiwaArmNumJoints, 0.);
        iiwa_state.joint_position_commanded.resize(kIiwaArmNumJoints, 0.);
        iiwa_state.joint_position_ipo.resize(kIiwaArmNumJoints, 0.);
        iiwa_state.joint_torque_measured.resize(kIiwaArmNumJoints, 0.);
        iiwa_state.joint_torque_commanded.resize(kIiwaArmNumJoints, 0.);
        iiwa_state.joint_torque_external.resize(kIiwaArmNumJoints, 0.);
        
        //////////////// !!!!!!!!!! /////////////////////
        // Warning: just for visualization since the gripper_position_output_port has been depricated in latest drake update
        // actual_position_mm = finger1 -- negative
        // actual_speed_mm_per_s = finger2 -- positive
        // maximum width = 110

        wsg_status.actual_position_mm = -25;
        wsg_status.actual_speed_mm_per_s = 25;
        
        object_state.joint_position_measured.resize(7, 0.);   
        object_state.joint_velocity_estimated.resize(7, 0.);
        object_state.joint_position_commanded.resize(7, 0.);
        object_state.joint_position_ipo.resize(7, 0.);
        object_state.joint_torque_measured.resize(7, 0.);
        object_state.joint_torque_commanded.resize(7, 0.);
        object_state.joint_torque_external.resize(7, 0.);

        drake::log()->info("Publishing trajectory to visualizer");
        plan_finished_ = false;
        // DRAKE_ASSERT(publish_rate )
        // unsigned int cur_step = int((robot_time_.utime / 1000)*(0.001*publish_rate/(time_step/InterpolationScale)));
        // cout << "starting time: " << cur_step << endl;
        // bool start_publish = false;
        // unsigned int start_time;

        while(true){
            while (0 == lcm_.handleTimeout(10) || iiwa_state.utime == -1 
            || plan_finished_) { }

            // if(!start_publish){
            //   start_time = robot_time_.utime;     
            //   cout << "start_time: " << start_time << endl; 
            //   start_publish = true;
            // }

            // Update status time to simulation time
            // Note: utime is in microseconds
            iiwa_state.utime = robot_time_.utime;
            wsg_status.utime = robot_time_.utime;
            // step_ = int((robot_time_.utime / 1000)*(kIiwaLcmStatusPeriod/(time_step/InterpolationScale)));
            step_ = int(((robot_time_.utime) / 1000)*(0.001*realtime_rate/(time_step/InterpolationScale)));
            std::cout << step_ << std::endl;
            
            if(step_ >= N*InterpolationScale)
            {
                drake::log()->info("Interpolated trajectory has been published");
                plan_finished_ = true;
                break;
            }

            // pass the interpolated traj to lcm
            for (int32_t j=0; j < iiwa_state.num_joints; ++j) { 
                iiwa_state.joint_position_measured[j] = joint_state_traj_interp[step_][13 + j];
            }

            lcm_.publish(kLcmStatusChannel, &iiwa_state);
            

            for (int joint = 0; joint < 7; joint++) 
            {
                object_state.joint_position_measured[joint] = joint_state_traj_interp[step_][joint];
            }

            lcm_.publish(kLcmObjectStatusChannel, &object_state);
            lcm_.publish(kLcmSchunkStatusChannel, &wsg_status);
        }

    };

    void saveVector(const Eigen::MatrixXd & _vec, const char * _name){
        std::string _file_name = UDP_TRAJ_DIR;
        _file_name += _name;
        _file_name += ".csv";
        clean_file(_name, _file_name);

        std::ofstream save_file;
        save_file.open(_file_name, std::fstream::app);
        for (int i(0); i < _vec.rows(); ++i){
            save_file<<_vec(i,0)<< "\t";
        }
        save_file<<"\n";
        save_file.flush();
        save_file.close();
    }

    void saveValue(double _value, const char * _name){
        std::string _file_name = UDP_TRAJ_DIR;
        _file_name += _name;
        _file_name += ".csv";
        clean_file(_name, _file_name);

        std::ofstream save_file;
        save_file.open(_file_name, std::fstream::app);
        save_file<<_value <<"\n";
        save_file.flush();
    }

    void clean_file(const char * _file_name, std::string & _ret_file){
        std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
        if(gs_fileName_string.end() == iter){
            gs_fileName_string.push_back(_file_name);
            remove(_ret_file.c_str());
        }
    }

private:
    void HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
        robot_time_ = *robot_time;
    }

};

int do_main_kkt(){
    TrajOptPublisher pub;
    fullstateVec_t xinit,xgoal;
    double time_horizon = 1.0;
    double time_step = 0.005;
    double realtime_rate = 0.2;

    std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string action = "push";
    std::vector<Eigen::VectorXd> ik_res;
    if (action.compare("push")==0){
        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
        //waypoint (0)
        ConstraintRelaxingIk::IkCartesianWaypoint wp0;
        const Eigen::Vector3d xyz0(
            0.0,
            0.55,
            0.0816099
        );
        const math::RollPitchYaw<double> rpy0(
            // 0,
            // 1.57079632679,
            // 1.57079632679
            1.42092e-12,
            0.0292037,
            4.26875e-12
        );
        // rpy0.To
        wp0.pose.set_translation(xyz0);
        wp0.pose.set_rotation(rpy0);
        wp0.constrain_orientation = true;
        wp_vec.push_back(wp0);

        // waypoint (1)
        ConstraintRelaxingIk::IkCartesianWaypoint wp1;
        const Eigen::Vector3d xyz1(
            0.5,
            0.55,
            0.0816099
        );
        const math::RollPitchYaw<double> rpy1(
            // 0,
            // 1.57079632679,
            // 1.57079632679
            1.42092e-12,
            0.0292037,
            4.26875e-12
        );

        wp1.pose.set_translation(xyz1);
        wp1.pose.set_rotation(rpy1);
        wp1.constrain_orientation = true;
        wp_vec.push_back(wp1);

        Eigen::VectorXd iiwa_q(7);
        iiwa_q << -0.133372, 0.251457, -0.0461879, -1.21048, 0.0324702, 0.928553, -0.190112; //warm-start for grasping from top

        ConstraintRelaxingIk ik(
            kIiwaUrdf,
            FLAGS_ee_name
        );

        if(!ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)){
        cout << "infeasible" << endl;
        }
        for (unsigned int y=0; y<ik_res.size(); y++){
        cout << ik_res[y].transpose() << endl;
        }
        cout << "Finished" << endl;
    }
    else{
        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> wp_vec;
        //waypoint (0)
        ConstraintRelaxingIk::IkCartesianWaypoint wp0;
        const Eigen::Vector3d xyz0(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            0.0,
            0.3
        );
        const math::RollPitchYaw<double> rpy0(
            0,
            1.57079632679,
            1.57079632679
        );
        // rpy0.To
        wp0.pose.set_translation(xyz0);
        wp0.pose.set_rotation(rpy0);
        wp0.constrain_orientation = true;
        wp_vec.push_back(wp0);

        // waypoint (1)
        ConstraintRelaxingIk::IkCartesianWaypoint wp1;
        const Eigen::Vector3d xyz1(
            (FLAGS_belt_width+FLAGS_table_width)/2+0.03-FLAGS_default_iiwa_x,
            -0.3,
            0.8
        );
        const math::RollPitchYaw<double> rpy1(
            0,
            1.57079632679,
            1.57079632679
        );

        wp1.pose.set_translation(xyz1);
        wp1.pose.set_rotation(rpy1);
        wp1.constrain_orientation = true;
        wp_vec.push_back(wp1);

        Eigen::VectorXd iiwa_q(7);
        iiwa_q << -0.133372, 0.251457, -0.0461879, -1.21048, 0.0324702, 0.928553, -0.190112; //warm-start for grasping from top

        ConstraintRelaxingIk ik(
            kIiwaUrdf,
            FLAGS_ee_name
        );

        if(!ik.PlanSequentialTrajectory(wp_vec, iiwa_q, &ik_res)){
        cout << "infeasible" << endl;
        }
        for (unsigned int y=0; y<ik_res.size(); y++){
        cout << ik_res[y].transpose() << endl;
        }
        cout << "Finished" << endl;

        // object initial and final states
        Vector3d object_pos_init = xyz0;
        object_pos_init(2, 0) -= 0.21;
        Vector3d object_pos_goal = xyz1;
        object_pos_goal(2, 0) -= 0.21;

    }
    xinit.setZero();
    // xinit.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, 0, 0.09, 0, 0, 0, 0, 0, 0;
    xinit.topRows(13) << 1, 0, 0, 0, 
    0.26, 0.55, 0.09, 0, 0, 0, 0, 0, 0;
    xinit.middleRows<7>(13) = ik_res[1];

    xgoal.setZero();
    // xgoal.topRows(13) << 1, 0, 0, 0, 
    // (FLAGS_belt_width+FLAGS_table_width)/2+0.033-FLAGS_default_iiwa_x, -0.3, 0.59, 0, 0, 0, 0, 0, 0;
    xgoal.topRows(13) << 1, 0, 0, 0, 
    0.76, 0.55, 0.09, 0, 0, 0, 0, 0, 0;
    xgoal.middleRows<7>(13) = ik_res[2];
    
    pub.Run_test(xinit, xgoal, time_horizon, time_step, realtime_rate, action);

    return 0;
}
}
}
}

int main(){
    return drake::traj_gen::kuka_iiwa_arm::do_main_kkt();
}
