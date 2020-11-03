#pragma once

#ifndef ILQR_KKT_TRK_SOLVER_H
#define ILQR_KKT_TRK_SOLVER_H

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/ilqr_kkt/kuka_arm_track_contact.h"
#include "drake/traj_gen/ilqr_kkt/cost_function_contact_track.h"
#include <numeric>
#include <sys/time.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Cholesky>
//#include <qpOASES.hpp>
//#include <qpOASES/QProblemB.hpp>

#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1

#ifndef DEBUG_ILQR
#define DEBUG_ILQR 1
// #else
//     #if PREFIX1(DEBUG_ILQR)==1
//     #define DEBUG_ILQR 1
//     #endif
#endif

#define TRACE(x) do { if (DEBUG_ILQR) printf(x);} while (0)

using namespace Eigen;
//USING_NAMESPACE_QPOASES

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
// namespace {

class ILQRSolver_TRK_Contact
{
public:
    struct traj
    {
        fullstateVecTab_t xList;
        commandVecTab_t uList;
        unsigned int iter;
        double finalCost;
        double finalGrad;
        double finalLambda;
        Eigen::VectorXd time_forward, time_backward, time_derivative; //computation time?
    };

    struct tOptSet {
        int n_hor;
        int debug_level;
        fullstateVec_t xInit;
        double new_cost, cost, dcost, lambda, dlambda, g_norm, expected;
        double **p;
        const double *alpha;
        int n_alpha;
        double lambdaMax;
        double lambdaMin;
        double lambdaInit;
        double dlambdaInit;
        double lambdaFactor;
        unsigned int max_iter;
        double tolGrad;
        double tolFun;
        double tolConstraint;
        double zMin;
        int regType;
        int iterations;
        int *log_linesearch;
        double *log_z;
        double *log_cost;
        double dV[2];
        
        double w_pen_l;
        double w_pen_f;
        double w_pen_max_l;
        double w_pen_max_f;
        double w_pen_init_l;
        double w_pen_init_f;
        double w_pen_fact1;
        double w_pen_fact2;
        
        int print;
        double print_head; // print headings every print_head lines
        double last_head;
        Eigen::VectorXd time_backward, time_forward, time_derivative;
        Eigen::VectorXd alphaList;
        // traj_t *nominal;
        // traj_t *candidates[NUMBER_OF_THREADS]; 
        // traj_t trajectories[NUMBER_OF_THREADS+1];
        // multipliers_t multipliers;
    };

public:
    ILQRSolver_TRK_Contact(KukaArm_TRK_Contact& iiwaDynamicModel, CostFunctionKukaArm_TRK_Contact& iiwaCostFunction, bool fullDDP=0,bool QPBox=0);
    fullstateVecTab_t xList; // vector/array of stateVec_t = basically knot config over entire time horizon
    commandVecTab_t uList;
    costVecTab_t costList;
private:
protected:
    // attributes //
public:
private:
    KukaArm_TRK_Contact* dynamicModel;
    CostFunctionKukaArm_TRK_Contact* costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    fullstateVec_t xInit; //matrix of <statesize, 1> = essentially a vector
    fullstateVec_t xgoal;
    unsigned int N;
    unsigned int iter;
    double dt;

    // stateVecTab_t xList; // vector/array of stateVec_t = basically knot config over entire time horizon
    // commandVecTab_t uList;

    fullstateVecTab_t xList_bar; 
    commandVecTab_t uList_bar;
    commandVecTab_t initCommand;

    commandVecTab_t uListFull;
    commandVecTab_t uList_bar_Full;
    commandVec_t u_NAN; //matrix of <commandsize, 1> = essentially a vector
    fullstateVecTab_t updatedxList;
    commandVecTab_t updateduList;
    fullstateVecTab_t FList;
   
    costVecTab_t costListNew;
    struct traj lastTraj;
    struct timeval tbegin_time_fwd, tend_time_fwd, tbegin_time_bwd, tend_time_bwd, tbegin_time_deriv, tend_time_deriv;

    fullstateVecTab_t Vx;
    fullstateMatTab_t Vxx;

    fullstateVec_t Qx;
    fullstateMat_t Qxx;
    commandVec_t Qu;
    commandMat_t Quu;
    commandMat_t QuuF;
    commandMat_t QuuInv;
    commandR_fullstateC_t Qux;
    commandVec_t k;
    commandR_fullstateC_t K;
    commandVecTab_t kList;
    commandR_fullstateC_tab_t KList;
    double alpha;

    fullstateMat_t lambdaEye;
    unsigned int backPassDone;
    unsigned int fwdPassDone;
    unsigned int initFwdPassDone;
    unsigned int diverge;

    /* QP variables */
    //QProblemB* qp;
    bool enableQPBox;
    bool enableFullDDP;
    commandMat_t H;
    commandVec_t g;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;
    commandVec_t lb;
    commandVec_t ub;
    // int nWSR;
    //real_t* xOpt;

    tOptSet Op;
    Eigen::Vector2d dV;
    bool debugging_print;    
    int newDeriv;
    double g_norm_i, g_norm_max, g_norm_sum;
    bool isUNan;
protected:
    // methods
public:
    void firstInitSolver(fullstateVec_t& iiwaxInit, fullstateVec_t& iiwaxDes, fullstateVecTab_t& x_bar, commandVecTab_t& u_bar, 
                    commandVecTab_t initialTorque, unsigned int& iiwaN, double& iiwadt, unsigned int& iiwamax_iter, double& iiwatolFun, double& iiwatolGrad);
    void solveTrajectory();
    void initializeTraj();
    void standardizeParameters(tOptSet *o);
    struct traj getLastSolvedTrajectory();
    void doBackwardPass();
    void doForwardPass();
    bool isPositiveDefinite(const commandMat_t & Quu); 
protected:
};

// }  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake

#endif // ILQRSOLVER_KKT_TRK_H

