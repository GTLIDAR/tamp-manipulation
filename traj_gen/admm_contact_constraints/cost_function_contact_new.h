#pragma once

#ifndef COSTFUNCTIONKUKAARM_CONTACT_H
#define COSTFUNCTIONKUKAARM_CONTACT_H

#include "drake/traj_gen/config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

class CostFunctionKukaArm_Contact
{
public:
    CostFunctionKukaArm_Contact(unsigned int N, std::string action_name);
private:
protected:
	fullstateMat_t Q;
	fullstateMat_t Qf;
	commandMat_t R;

	fullstateVec_t QDiagElementVec;
	fullstateVec_t QfDiagElementVec;
	commandVec_t RDiagElementVec;
	// double pos_scale;
    // double vel_scale;
    // double pos_f_scale;
    // double vel_f_scale;
    // double torqoe_scale;
    
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

#endif // COSTFUNCTIONKUKAARM_H
