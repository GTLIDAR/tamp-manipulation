#pragma once

#ifndef COSTFUNCTIONKUKAARM_H
#define COSTFUNCTIONKUKAARM_H

#include "drake/traj_gen/config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

class CostFunctionKukaArm_TRK
{
public:
    CostFunctionKukaArm_TRK(double pos_weight, double vel_weight, double torque_weight, unsigned int N);
private:
protected:
	stateMat_t Q;
	stateMat_t Qf;
    stateMat_t Rho_state;
    commandMat_t Rho_torque;
	commandMat_t R;

	stateVec_t QDiagElementVec;
	stateVec_t QfDiagElementVec;
	commandVec_t RDiagElementVec;
    stateVec_t Rho_state_DiagElementVec;
	commandVec_t Rho_torque_DiagElementVec;

	// double pos_scale;
    // double vel_scale;
    // double pos_f_scale;
    // double vel_f_scale;
    // double torque_scale;
    // double rho_pos_weight;
	// double rho_vel_weight;
    // double rho_torque_weight;

	stateVecTab_t cx_new;
	commandVecTab_t cu_new; 
	stateMatTab_t cxx_new; 
	commandR_stateC_tab_t cux_new; 
	commandMatTab_t cuu_new;
	double c_new;
    // attributes
public:
	stateMat_t& getQ();
	stateMat_t& getQf();
    stateMat_t& getRho_state();
	commandMat_t& getRho_torque();
	commandMat_t& getR();
	stateVecTab_t& getcx();
	commandVecTab_t& getcu();
	stateMatTab_t& getcxx();
	commandR_stateC_tab_t& getcux();
	commandMatTab_t& getcuu();
	double& getc();
	double pos_scale;
    double vel_scale;
    double pos_f_scale;
    double vel_f_scale;
    double torque_scale;
    double rho_pos_weight;
	double rho_vel_weight;
    double rho_torque_weight;

	// unsigned int N;
private:

protected:
    // methods
public:
private:
protected:
    // accessors
public:

};

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake

#endif // COSTFUNCTIONKUKAARM_H