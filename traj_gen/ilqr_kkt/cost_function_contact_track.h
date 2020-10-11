#pragma once

#ifndef COSTFUNCTIONKUKAARM_CONTACT_TRK_H
#define COSTFUNCTIONKUKAARM_CONTACT_TRK_H

#include "drake/traj_gen/config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

class CostFunctionKukaArm_TRK_Contact
{
public:
    CostFunctionKukaArm_TRK_Contact(double pos_obj_weight, double pos_iiwa_weight, 
                                    double vel_obj_weight, double vel_iiwa_weight,
                                    double torque_weight, unsigned int N);
private:
protected:
	fullstateMat_t Q;
	fullstateMat_t Qf;
    fullstateMat_t Rho_state;
    commandMat_t Rho_torque;
	commandMat_t R;

	fullstateVec_t QDiagElementVec;
	fullstateVec_t QfDiagElementVec;
	commandVec_t RDiagElementVec;
    fullstateVec_t Rho_state_DiagElementVec;
	commandVec_t Rho_torque_DiagElementVec;

	// double pos_scale;
    // double vel_scale;
    // double pos_f_scale;
    // double vel_f_scale;
    // double torque_scale;
    double rho_pos_obj_weight;
    double rho_vel_obj_weight;
    double rho_pos_iiwa_weight;
	double rho_vel_iiwa_weight;
    double rho_torque_weight;

    // double rho_pos_weight;
	// double rho_vel_weight;
    // double rho_torque_weight;

	fullstateVecTab_t cx_new;
	commandVecTab_t cu_new; 
	fullstateMatTab_t cxx_new; 
	commandR_fullstateC_tab_t cux_new; 
	commandMatTab_t cuu_new;
	double c_new;
    // attributes
public:
	fullstateMat_t& getQ();
	fullstateMat_t& getQf();
    fullstateMat_t& getRho_state();
	commandMat_t& getRho_torque();
	commandMat_t& getR();
	fullstateVecTab_t& getcx();
	commandVecTab_t& getcu();
	fullstateMatTab_t& getcxx();
	commandR_fullstateC_tab_t& getcux();
	commandMatTab_t& getcuu();
	double& getc();

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

#endif // COSTFUNCTIONKUKAARM_CONTACT_TRK_H