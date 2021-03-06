#pragma once

#ifndef KUKAARM_H
#define KUKAARM_H

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/cost_function_kuka_arm_track.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

// #include <mutex>
// std::mutex mtx;

#define pi 3.141592653

#ifndef DEBUG_KUKA_ARM
#define DEBUG_KUKA_ARM 1
#else
    #if PREFIX1(DEBUG_KUKA_ARM)==1
    #define DEBUG_KUKA_ARM 1
    #endif
#endif

#define TRACE_KUKA_ARM(x) do { if (DEBUG_KUKA_ARM) printf(x);} while (0)

using namespace Eigen;
using namespace std;

using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyForces;
using drake::multibody::Parser;
using drake::math::RigidTransformd;


namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

class KukaArm_TRK
{
public:
    KukaArm_TRK();
    KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, string action_name);
    KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, MultibodyPlant<double>* plant, string action_name);
    ~KukaArm_TRK(){};
private:
protected:
    // attributes
    unsigned int stateNb;
    unsigned int commandNb;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    stateMat_t fx;
    stateTens_t fxx;
    stateR_commandC_t fu;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;

    stateMatTab_t fxList;
    stateR_commandC_tab_t fuList;
    stateTensTab_t fxxList;
    stateTensTab_t fxuList;
    stateR_commandC_Tens_t fuuList;

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
    
    stateVec_t xgoal;
private:
    
    stateMat_half_t H, C; // inertial, corillois dynamics
    stateVec_half_t G; // gravity? what is this?
    stateR_half_commandC_t Bu; //input mapping
    stateVec_half_t velocity;
    stateVec_half_t accel;
    stateVec_t Xdot_new;
    stateVec_half_t vd;
    stateVecTab_half_t vd_thread;
    stateVecTab_t Xdot_new_thread;

    stateVec_t Xdot1, Xdot2, Xdot3, Xdot4;
    stateMat_t A1, A2, A3, A4, IdentityMat;
    stateR_commandC_t B1, B2, B3, B4;
    stateVec_t Xp, Xp1, Xp2, Xp3, Xp4, Xm, Xm1, Xm2, Xm3, Xm4;
    
    bool debugging_print;
    stateMat_t AA;
    stateR_commandC_t BB;
    stateMatTab_t A_temp;
    stateR_commandC_tab_t B_temp;
    
    MultibodyPlant<double>* plant_{};

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    std::vector<Eigen::VectorXd> q_thread, qd_thread;

    string action_name_;
protected:
    // methods
public:
    stateVec_t kuka_arm_dynamics(const stateVec_t& X, const commandVec_t& tau);
    void kuka_arm_dyn_cst_ilqr(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, CostFunctionKukaArm_TRK*& costFunction);
    void kuka_arm_dyn_cst_min_output(const int& nargout, const stateVec_t& xList_curr, const commandVec_t& uList_curr,  const stateVec_t& xList_cur_bar, const commandVec_t& uList_cur_bar, const bool& isUNan, stateVec_t& xList_next, CostFunctionKukaArm_TRK*& costFunction);
    stateVec_t update(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B);
    void grad(const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B);
    void hessian(const stateVec_t& X, const commandVec_t& U, stateTens_t& fxx_p, stateR_stateC_commandD_t& fxu_p, stateR_commandC_commandD_t& fuu_p);    
    struct timeprofile getFinalTimeProfile();

    unsigned int getStateNb();
    unsigned int getCommandNb();
    commandVec_t& getLowerCommandBounds();
    commandVec_t& getUpperCommandBounds();
    stateMatTab_t& getfxList();
    stateR_commandC_tab_t& getfuList();

    VectorXd quasiStatic(const stateVec_t& X0);
private:
protected:
        // accessors //
public:
};

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake

#endif // KUKAARM_H