#pragma once

#ifndef KUKAARM_TRACK_CONTACT_NEW_H
#define KUKAARM_TRACK_CONTACT_NEW_H

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/admm_contact_constraints/cost_function_contact_track_new.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/quaternion.h"

#include <cstdio>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

// #include <mutex>
// std::mutex mtx;

#define pi 3.141592653

#ifndef DEBUG_KUKA_ARM
#define DEBUG_KUKA_ARM 1
// #else
//     #if PREFIX1(DEBUG_KUKA_ARM)==1
//     #define DEBUG_KUKA_ARM 1
//     #endif
#endif

#define TRACE_KUKA_ARM(x) do { if (DEBUG_KUKA_ARM) printf(x);} while (0)

using namespace Eigen;
using namespace std;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyForces;
using drake::multibody::ModelInstanceIndex;
using drake::math::RollPitchYaw;
using drake::math::CalculateQuaternionDtFromAngularVelocityExpressedInB;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::SpatialAcceleration;


namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

class KukaArm_TRK_Contact_new
{
public:
    KukaArm_TRK_Contact_new();
    KukaArm_TRK_Contact_new(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal, string action_name);
    KukaArm_TRK_Contact_new(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal, MultibodyPlant<double>* plant, string action_name);
    ~KukaArm_TRK_Contact_new(){};
private:
protected:
    // attributes
    unsigned int stateNb;
    unsigned int commandNb;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    fullstateMat_t fx;
    fullstateTens_t fxx;
    fullstateR_commandC_t fu;
    fullstateR_commandC_commandD_t fuu;
    fullstateR_fullstateC_commandD_t fxu;
    fullstateR_commandC_fullstateD_t fux;

    fullstateMatTab_t fxList;
    fullstateR_commandC_tab_t fuList;
    fullstateTensTab_t fxxList;
    fullstateTensTab_t fxuList;
    fullstateR_commandC_Tens_t fuuList;

    forceR_fullstateC_t gx; 
    forceR_commandC_t gu;   

    forceR_fullstateC_tab_t gxList;
    forceR_commandC_tab_t guList;

public:
    struct timeprofile
    {
        double time_period1, time_period2, time_period3, time_period4;
        unsigned int counter0_, counter1_, counter2_;
    };

private:
    double dt;
    unsigned int N;
    bool initial_phase_flag_;
    struct timeprofile finalTimeProfile;
    struct timeval tbegin_period, tend_period, tbegin_period4, tend_period4; //tbegin_period2, tend_period2, tbegin_period3, tend_period3, 

public:
    static const double mc, mp, l, g;
    
    //######
    unsigned int globalcnt;
    //######
    
    fullstateVec_t xgoal;
private:
    
    stateVec_half_t velocity;
    stateVec_half_t accel;
    fullstateVec_t Xdot_new;
    stateVec_half_t vd;
    stateVecTab_half_t vd_thread;
    fullstateVecTab_t Xdot_new_thread;

    fullstateVec_t Xdot1, Xdot2, Xdot3, Xdot4;
    fullstateMat_t A1, A2, A3, A4, IdentityMat;
    fullstateR_commandC_t B1, B2, B3, B4;
    fullstateVec_t Xp, Xp1, Xp2, Xp3, Xp4, Xm, Xm1, Xm2, Xm3, Xm4;
    
    bool debugging_print;
    fullstateMat_t AA;
    fullstateR_commandC_t BB;
    forceR_fullstateC_t CC;
    forceR_commandC_t DD;
    fullstateMatTab_t A_temp;
    fullstateR_commandC_tab_t B_temp;
    forceR_fullstateC_tab_t C_temp;
    forceR_commandC_tab_t D_temp;
    
    MultibodyPlant<double>* plant_{};

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    std::vector<Eigen::VectorXd> q_thread, qd_thread;

    string action_name_;
protected:
    // methods
public:
    fullstateVec_t kuka_arm_dynamics(const fullstateVec_t& X, const commandVec_t& tau, forceVec_t& force);
    void kuka_arm_dyn_cst_ilqr(const fullstateVecTab_t& xList, const commandVecTab_t& uList, const forceVecTab_t& forceList, const fullstateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, const forceVecTab_t& forceList_bar, CostFunctionKukaArm_TRK_Contact_new*& costFunction);
    void kuka_arm_dyn_cst_min_output(const fullstateVec_t& xList_curr, const commandVec_t& uList_curr, forceVec_t& forceList_curr, const fullstateVec_t& xList_cur_bar, const commandVec_t& uList_cur_bar, forceVec_t& forceList_cur_bar, const bool& isUNan, fullstateVec_t& xList_next, CostFunctionKukaArm_TRK_Contact_new*& costFunction);
    fullstateVec_t update(const fullstateVec_t& X, const commandVec_t& U, forceVec_t& force);
    void grad(const fullstateVec_t& X, const commandVec_t& U, fullstateMat_t& A, fullstateR_commandC_t& B, forceR_fullstateC_t& C, forceR_commandC_t& D);   
    // void hessian(const fullstateVec_t& X, const commandVec_t& U, fullstateTens_t& fxx_p, fullstateR_fullstateC_commandD_t& fxu_p, fullstateR_commandC_commandD_t& fuu_p);    
    struct timeprofile getFinalTimeProfile();

    unsigned int getStateNb();
    unsigned int getCommandNb();
    commandVec_t& getLowerCommandBounds();
    commandVec_t& getUpperCommandBounds();
    fullstateMatTab_t& getfxList();
    fullstateR_commandC_tab_t& getfuList();
    forceR_fullstateC_tab_t& getgxList();
    forceR_commandC_tab_t& getguList();
private:
protected:
        // accessors //
public:
};

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake

#endif // KUKAARM_H