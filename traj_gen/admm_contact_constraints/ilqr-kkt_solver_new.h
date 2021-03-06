#pragma once

#ifndef ILQR_KKT_SOLVER_NEW_H
#define ILQR_KKT_SOLVER_NEW_H

#include "drake/traj_gen/config.h"
#include "drake/traj_gen/admm_contact_constraints/kuka_arm_contact_new.h"
#include "drake/traj_gen/admm_contact_constraints/cost_function_contact_new.h"
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

class ILQR_KKTSolver_new
{
public:
    struct traj
    {
        fullstateVecTab_t xList;
        commandVecTab_t uList;
        forceVecTab_t forceList;
        unsigned int iter;
        double finalCost;
        double finalGrad;
        double finalLambda;
        Eigen::VectorXd time_forward, time_backward, time_derivative; //computation time?
        unsigned int FK_count;
        double time_qp;
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
    ILQR_KKTSolver_new(KukaArm_Contact_new& iiwaDynamicModel, CostFunctionKukaArm_Contact_new& iiwaCostFunction, bool fullDDP=0,bool QPBox=0);
private:
protected:
    // attributes //
public:
private:
    KukaArm_Contact_new* dynamicModel;
    CostFunctionKukaArm_Contact_new* costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    fullstateVec_t xInit; //matrix of <statesize, 1> = essentially a vector
    fullstateVec_t xgoal;
    unsigned int N;
    unsigned int iter;
    double dt;
    commandVecTab_t initCommand;

    fullstateVecTab_t xList; // vector/array of stateVec_t = basically knot config over entire time horizon
    commandVecTab_t uList;
    commandVecTab_t uListFull;
    forceVecTab_t forceList;
    forceVecTab_t forceListFull;
    commandVec_t u_NAN; //matrix of <commandsize, 1> = essentially a vector
    fullstateVecTab_t updatedxList;
    commandVecTab_t updateduList;
    forceVecTab_t updatedforceList;
    costVecTab_t costList;
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
    void firstInitSolver(fullstateVec_t& iiwaxInit, fullstateVec_t& iiwaxDes, commandVecTab_t initialTorque, unsigned int& iiwaN,
                    double& iiwadt, unsigned int& iiwamax_iter, double& iiwatolFun, double& iiwatolGrad);
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

#endif // ILQRSOLVER_H