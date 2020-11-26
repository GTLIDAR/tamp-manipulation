#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <numeric>
#include <sys/time.h>

#define UDP_BACKWARD_INTEGRATION_METHOD 0 // 1: 4^th-order RK, 2: simple Euler method, 3: 3^rd-order RK with FOH on u (same as dircol)
#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1
#define INCLUDE_OBJECT 1
#define DIRECT_INVERSE 0
#define WHOLE_BODY 1
#define VISUALIZE 1
#define NUM_CONTACT_POINTS 2

#define MULTI_THREAD 0
#if MULTI_THREAD
#include <thread>
#define NUMBER_OF_THREAD 1 //12
#endif

#define stateSize 14
#define commandSize 7
#define statecommandSize 21

#define fullstateSize 27 // object: 7+6; kuka: 7+7 (the joints for wsg cannot be optimized) 
#define fullstatecommandSize 34 // add the size of torques

#define forceSize 6*NUM_CONTACT_POINTS

// #define TimeHorizon 2
// #define TimeStep 0.005
// #define NumberofKnotPt TimeHorizon/TimeStep
#define InterpolationScale 10 //0.01/1e-3
const int32_t kNumJoints = 7;

#define BOXWEIGHT 0.122

#define UDP_TRAJ_DIR "/home/ziyi/code/drake/traj_gen/trajectory_data/"
const char* const kLcmQueryResultsChannel = "TREE_SEARCH_QUERY_RESULTS";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmObjectStatusChannel = "OBJECT_STATUS";
const char* const kLcmSchunkStatusChannel = "WSG_STATUS";
const char* const kLcmTimeChannel = "IIWA_TIME";

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
namespace {

// typedef for stateSize types
typedef Eigen::Matrix<double,stateSize,1> stateVec_t;                       // stateSize x 1
typedef Eigen::Matrix<double,1,stateSize> stateVecTrans_t;                  // 1 x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateMat_t;               // stateSize x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

typedef Eigen::Matrix<double,fullstateSize,1> fullstateVec_t;                       // fullstateSize x 1
typedef Eigen::Matrix<double,1,fullstateSize> fullstateVecTrans_t;                  // 1 x fullstateSize
typedef Eigen::Matrix<double,fullstateSize,fullstateSize> fullstateMat_t;               // fullstateSize x fullstateSize
typedef Eigen::Matrix<double,fullstateSize,fullstateSize> fullstateTens_t[fullstateSize];   // fullstateSize x fullstateSize x fullstateSize

// typedef for commandSize types
typedef Eigen::Matrix<double,commandSize,1> commandVec_t;                           // commandSize x 1
typedef Eigen::Matrix<double,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize

// typedef for forceSize types
typedef Eigen::Matrix<double,forceSize,1> forceVec_t;                       // forceSize x 1
typedef Eigen::Matrix<double,1,forceSize> forceVecTrans_t;                  // 1 x forceSize
typedef Eigen::Matrix<double,forceSize,forceSize> forceMat_t;               // forceSize x forceSize
typedef Eigen::Matrix<double,forceSize,forceSize> forceTens_t[forceSize];   // forceSize x forceSize x forceSize

// typedef for mixed stateSize and commandSize types
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_t;                          // stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_t;                          // commandSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateR_stateC_commandD_t[commandSize];        // stateSize x stateSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize
typedef Eigen::Matrix<double,stateSize+commandSize,1> stateAug_t;                               // stateSize + commandSize x 1
typedef Eigen::Matrix<double,1,1> scalar_t;                                                     // 1 x 1
typedef Eigen::Matrix<double,stateSize+commandSize,1> projStateAndCommand_t;                    // 21 x 1

typedef Eigen::Matrix<double,fullstateSize,commandSize> fullstateR_commandC_t;                          // fullstateSize x commandSize
typedef Eigen::Matrix<double,fullstateSize,commandSize> fullstateR_commandC_fullstateD_t[stateSize];        // fullstateSize x commandSize x fullstateSize
typedef Eigen::Matrix<double,fullstateSize,commandSize> fullstateR_commandC_commandD_t[commandSize];    // fullstateSize x commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,fullstateSize> commandR_fullstateC_t;                          // commandSize x fullstateSize
typedef Eigen::Matrix<double,commandSize,fullstateSize> commandR_fullstateC_fullstateD_t[stateSize];        // commandSize x fullstateSize x fullstateSize
typedef Eigen::Matrix<double,commandSize,fullstateSize> commandR_fullstateC_commandD_t[commandSize];    // commandSize x fullstateSize x commandSize
typedef Eigen::Matrix<double,fullstateSize,fullstateSize> fullstateR_fullstateC_commandD_t[commandSize];        // fullstateSize x fullstateSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandR_commandC_fullstateD_t[stateSize];    // commandSize x commandSize x fullstateSize
typedef Eigen::Matrix<double,fullstateSize+commandSize,1> fullstateAug_t;                               // fullstateSize + commandSize x 1
typedef Eigen::Matrix<double,fullstateSize+commandSize,1> projfullStateAndCommand_t;                    // 34 x 1

// typedef for mixed force related matrices
typedef Eigen::Matrix<double,forceSize,commandSize> forceR_commandC_t;                          // forceSize x commandSize gu
typedef Eigen::Matrix<double,forceSize,fullstateSize> forceR_fullstateC_t;                          // forceSize x fullstateSize gx
typedef Eigen::Matrix<double,forceSize,fullstateSize> forceR_fullstateC_fullstateD_t[fullstateSize];                          // forceSize x fullstateSize x fullstateSize gxx
typedef Eigen::Matrix<double,forceSize,commandSize> forceR_commandC_fullstateD_t[fullstateSize];                          // forceSize x commandSize x fullstateSize gux
typedef Eigen::Matrix<double,forceSize,fullstateSize> forceR_fullstateC_commandD_t[commandSize];                          // forceSize x fullstateSize x commandSize gxu
typedef Eigen::Matrix<double,forceSize,commandSize> forceR_commandC_commandD_t[commandSize];                          // forceSize x commandSize x commandSize guu




// typedef for half commandSize and stateSize types
typedef Eigen::Matrix<double,stateSize/2,1> stateVec_half_t;                                    // stateSize/2 x 1
typedef Eigen::Matrix<double,stateSize/2,stateSize/2> stateMat_half_t;                          // stateSize/2 x stateSize/2
typedef Eigen::Matrix<double,stateSize/2,commandSize> stateR_half_commandC_t;                   // stateSize/2 x commandSize

// typedef for vectorized state and command matrix (over the horizon)
typedef std::vector<stateVec_t> stateVecTab_t;
typedef std::vector<double> costVecTab_t;
typedef std::vector<commandVec_t> commandVecTab_t;
typedef std::vector<stateMat_t> stateMatTab_t;
typedef std::vector<commandMat_t> commandMatTab_t;
typedef std::vector<stateR_commandC_t> stateR_commandC_tab_t;
typedef std::vector<commandR_stateC_t> commandR_stateC_tab_t;
typedef std::vector<stateVec_half_t> stateVecTab_half_t;
typedef std::vector<projStateAndCommand_t> projStateAndCommandTab_t;

typedef std::vector<fullstateVec_t> fullstateVecTab_t;
typedef std::vector<fullstateMat_t> fullstateMatTab_t;
typedef std::vector<fullstateR_commandC_t> fullstateR_commandC_tab_t;
typedef std::vector<commandR_fullstateC_t> commandR_fullstateC_tab_t;
typedef std::vector<projfullStateAndCommand_t> projfullStateAndCommandTab_t;

// typedef for force related matrices (over the horizon)
typedef std::vector<forceVec_t> forceVecTab_t;
typedef std::vector<forceMat_t> forceMatTab_t;
typedef std::vector<forceR_fullstateC_t> forceR_fullstateC_tab_t;
typedef std::vector<forceR_commandC_t> forceR_commandC_tab_t;

//typedef std::vector<stateTens_t> stateTensTab_t;
typedef std::vector<std::vector<stateMat_t> > stateTensTab_t;
typedef std::vector<std::vector<stateR_commandC_t> > stateR_commandC_Tens_t;

typedef std::vector<std::vector<fullstateMat_t> > fullstateTensTab_t;
typedef std::vector<std::vector<fullstateR_commandC_t> > fullstateR_commandC_Tens_t;

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake

#endif // CONFIG_H