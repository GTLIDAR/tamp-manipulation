#include "drake/traj_gen/cost_function_kuka_arm_track.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
	
CostFunctionKukaArm_TRK::CostFunctionKukaArm_TRK(double pos_weight, double vel_weight, double torque_weight, unsigned int N)
{    
    pos_scale = 5;
    vel_scale = 5;
    pos_f_scale = 1000;
    vel_f_scale = 10;
    torque_scale = 1;//100;
    rho_pos_weight = pos_weight;
    rho_vel_weight = vel_weight;
    rho_torque_weight = torque_weight;

    // penalize accleration, penalize total variation

    // initial, final costs (pos ,vel)
    // torque cost
    // l = sigma(xQx+uRu) + xfQfxf
    QDiagElementVec << pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100,  
                        vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10;
    QfDiagElementVec << pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0,
                        vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0;
    RDiagElementVec << torque_scale*0.005, torque_scale*0.005, torque_scale*0.007, torque_scale*0.007, torque_scale*0.02, torque_scale*0.02, torque_scale*0.05;
    Rho_state_DiagElementVec << rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight,
                        rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight;
                        // 2.5*rho_vel_weight, 2.5*rho_vel_weight, 1.8*rho_vel_weight, 1.25*rho_vel_weight, 1.15*rho_vel_weight, rho_vel_weight, rho_vel_weight; //2.5, 2.5, 1.8, 1.25, 1.15, 1.0, 1.0
    Rho_torque_DiagElementVec << rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight;

    Q = QDiagElementVec.asDiagonal();
    Qf = QfDiagElementVec.asDiagonal();
    R = RDiagElementVec.asDiagonal();
    Rho_state = Rho_state_DiagElementVec.asDiagonal();
    Rho_torque = Rho_torque_DiagElementVec.asDiagonal();

    // TimeHorizon = total time 
    // TimeStep = time between two timesteps
    // N = number of knot
    // N = TimeHorizon/TimeStep;
    cx_new.resize(N+1);
    cu_new.resize(N+1);
    cxx_new.resize(N+1);
    cux_new.resize(N+1);
    cuu_new.resize(N+1);
}

stateMat_t& CostFunctionKukaArm_TRK::getQ()
{
    return Q;
}

stateMat_t& CostFunctionKukaArm_TRK::getRho_state()
{
    return Rho_state;
}

commandMat_t& CostFunctionKukaArm_TRK::getRho_torque()
{
    return Rho_torque;
}

stateMat_t& CostFunctionKukaArm_TRK::getQf()
{
    return Qf;
}

commandMat_t& CostFunctionKukaArm_TRK::getR()
{
    return R;
}

stateVecTab_t& CostFunctionKukaArm_TRK::getcx()
{
    return cx_new;
}

commandVecTab_t& CostFunctionKukaArm_TRK::getcu()
{
    return cu_new;
}

stateMatTab_t& CostFunctionKukaArm_TRK::getcxx()
{
    return cxx_new;
}

commandR_stateC_tab_t& CostFunctionKukaArm_TRK::getcux()
{
    return cux_new;
}

commandMatTab_t& CostFunctionKukaArm_TRK::getcuu()
{
    return cuu_new;
}

double& CostFunctionKukaArm_TRK::getc()
{
    return c_new;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake