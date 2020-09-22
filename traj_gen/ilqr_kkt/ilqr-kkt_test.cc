#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_manipulator_traj.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

#include "drake/lcmt_generic_string_msg.hpp"

#include "drake/traj_gen/ilqr_kkt/config-kkt.h"
#include "drake/traj_gen/ilqr_kkt/ilqr-kkt_solver.h"
#include "drake/traj_gen/ilqr_kkt/kuka_arm_contact.h"

#include "drake/lcmt_motion_plan_query.hpp"

using namespace std;
using namespace Eigen;

#define useILQRSolver 1
#define useUDPSolver 0

/* DDP trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using multibody::ModelInstanceIndex;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyForces;
using multibody::BodyIndex;

class ilqr_kkt_test
{
private:
    stateVecTab_t joint_state_traj;
    commandVecTab_t torque_traj;
    stateVecTab_t joint_state_traj_interp;
    commandVecTab_t torque_traj_interp;

public:
    void Run_test(stateVec_t xinit, stateVec_t xgoal, double time_horizon, double time_step){
        struct timeval tbegin,tend;
        double texec = 0.0;
        double dt = time_step;
        unsigned int N = int(time_horizon/time_step);
        double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
        double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
        unsigned int iterMax = 10; //100;    

        ILQR_KKTSolver::traj lastTraj;
        //=============================================
        // Build wholebody and pass over to kukaArm
        std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
        std::string schunkPath = 
          FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
        std::string connectorPath = 
          FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");
        const std::string box_sdf_path0 = "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf";

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
        RigidTransformd X_EG(RollPitchYaw<double>(0, 0, M_PI_2),
                                    Vector3d(0, 0, 0.0175));
        plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

        const ModelInstanceIndex object_model =
        parser.AddModelFromFile(FindResourceOrThrow(box_sdf_path0), "object");

        plant_.Finalize();

        // auto context_ptr = plant_.CreateDefaultContext();
        // auto context = context_ptr.get();

        // VectorXd q_v_iiwa(14);
        // q_v_iiwa.setZero();
        // q_v_iiwa.head(7) = xinit;
        // plant_.SetPositionsAndVelocities(context, iiwa_model, q_v_iiwa);

        // MatrixXd M_(plant_.num_velocities(), plant_.num_velocities());
        // plant_.CalcMassMatrix(*context, &M_);

        // VectorXd gtau_wb = plant_.CalcGravityGeneralizedForces(*context);

        // cout << "bias total" << endl << gtau_wb << endl;
        commandVecTab_t u_0;
        u_0.resize(N);
        for(unsigned i=0;i<N;i++){
        //   u_0[i].head(7) = gtau_wb;
          u_0[i].setZero();
        }
        //======================================
        KukaArm_Contact KukaArmModel(dt, N, xgoal, &plant_);
        CostFunctionKukaArm_Contact costKukaArm(N);
        ILQR_KKTSolver testSolverKukaArm(KukaArmModel,costKukaArm,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverKukaArm.firstInitSolver(xinit, xgoal, u_0, N, dt, iterMax, tolFun, tolGrad);     

        // run one or multiple times and then average
        unsigned int Num_run = 1;
        gettimeofday(&tbegin,NULL);
        for(unsigned int i=0;i<Num_run;i++) testSolverKukaArm.solveTrajectory();
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
        for(unsigned int i=0;i<stateSize;i++){
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


        cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList[N].transpose() << endl;
        cout << "lastTraj.uList[" << N << "]:" << lastTraj.uList[N].transpose() << endl;

        cout << "lastTraj.xList[0]:" << lastTraj.xList[0].transpose() << endl;
        cout << "lastTraj.uList[0]:" << lastTraj.uList[0].transpose() << endl;

        auto context_ptr = plant_.CreateDefaultContext();
        auto context = context_ptr.get();
        for(unsigned int i=0;i<=N;i++){
          // Do the forward kinamtics for the EE to check the performance
          auto rpy = math::RollPitchYawd(Eigen::Vector3d(0, 0, 0));
          auto xyz = Eigen::Vector3d(0, 0, 0);
          math::RigidTransform<double> X_WO(math::RotationMatrix<double>(rpy), xyz);
          plant_.SetFreeBodyPoseInWorldFrame(context, plant_.GetBodyByName("base_link_cracker", object_model), X_WO);
          plant_.SetPositions(context, iiwa_model, lastTraj.xList[i].topRows(7));
          plant_.SetVelocities(context, iiwa_model, lastTraj.xList[i].bottomRows(7));
          const auto& X_WB_all = plant_.get_body_poses_output_port().Eval<std::vector<math::RigidTransform<double>>>(*context);
          const BodyIndex ee_body_index = plant_.GetBodyByName("iiwa_link_ee_kuka", iiwa_model).index();
          const math::RigidTransform<double>& X_Wee = X_WB_all[ee_body_index];
          cout << "ee[" << i << "]:" << X_Wee.translation().transpose() << endl;
        }
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
        // traj_knot_number_ = 0;

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

        // return *ptr;
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

};

int do_main_kkt(){
    ilqr_kkt_test test;
    stateVec_t xinit,xgoal;
    double time_horizon = 2.0;
    double time_step = 0.01;
    xinit << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    xgoal << 1.0,1.0,1.0,1.0,1.0,1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    test.Run_test(xinit, xgoal, time_horizon, time_step);

    return 0;
}
}
}
}

int main(){
    return drake::traj_gen::kuka_iiwa_arm::do_main_kkt();
}
