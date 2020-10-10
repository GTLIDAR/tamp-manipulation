#include "drake/traj_gen/ilqr_kkt/cost_function_contact_track.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
	
CostFunctionKukaArm_TRK_Contact::CostFunctionKukaArm_TRK_Contact(double pos_obj_weight, double pos_iiwa_weight, 
                                                                 double vel_obj_weight, double vel_iiwa_weight,
                                                                 double torque_weight, unsigned int N)
{   
    if(INCLUDE_OBJECT){ 
        // The dimension is 27 in this case
        double pos_obj_scale = 10;
        double vel_obj_scale = 10;
        double pos_obj_f_scale = 100;//0.001;
        double vel_obj_f_scale = 100;//10;
        double torqoe_scale = 1;//100;
        
        double pos_iiwa_scale = 10;
        double vel_iiwa_scale = 10;
        double pos_iiwa_f_scale = 100;//0.001;
        double vel_iiwa_f_scale = 100;//10;
        
        rho_pos_obj_weight = pos_obj_weight;
        rho_vel_obj_weight = vel_obj_weight;

        rho_pos_iiwa_weight = pos_iiwa_weight;
        rho_vel_iiwa_weight = vel_iiwa_weight;
        rho_torque_weight = torque_weight;

        QDiagElementVec << pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100,
                            vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10,
                            pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100,
                            vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10;
        QfDiagElementVec << pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0,
                            vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0,
                            pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0,
                            vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0;
        RDiagElementVec << torqoe_scale*0.005, torqoe_scale*0.005, torqoe_scale*0.007, torqoe_scale*0.007, torqoe_scale*0.02, torqoe_scale*0.02, torqoe_scale*0.05;

        Rho_state_DiagElementVec << rho_pos_obj_weight, rho_pos_obj_weight, rho_pos_obj_weight, rho_pos_obj_weight, rho_pos_obj_weight, rho_pos_obj_weight, rho_pos_obj_weight, 
                            rho_vel_obj_weight, rho_vel_obj_weight, rho_vel_obj_weight, rho_vel_obj_weight, rho_vel_obj_weight, rho_vel_obj_weight, 
                            rho_pos_iiwa_weight, rho_pos_iiwa_weight, rho_pos_iiwa_weight, rho_pos_iiwa_weight, rho_pos_iiwa_weight, rho_pos_iiwa_weight, rho_pos_iiwa_weight,
                            rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight;
        Rho_torque_DiagElementVec << rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight;
    }
    // else{
    //     double pos_scale = 10;
    //     double vel_scale = 10;
    //     double pos_f_scale = 1000;//0.001;
    //     double vel_f_scale = 10;//10;
    //     double torqoe_scale = 1;//100;
    //     rho_pos_weight = pos_weight;
    //     rho_vel_weight = vel_weight;
    //     rho_torque_weight = torque_weight;

    //     QDiagElementVec << pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100,
    //                         vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10;
    //     QfDiagElementVec << pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0,
    //                         vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0;
    //     RDiagElementVec << torqoe_scale*0.005, torqoe_scale*0.005, torqoe_scale*0.007, torqoe_scale*0.007, torqoe_scale*0.02, torqoe_scale*0.02, torqoe_scale*0.05;
    //     Rho_state_DiagElementVec << rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight, rho_pos_weight,
    //                         rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight, rho_vel_weight;
    //     Rho_torque_DiagElementVec << rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight;
    // }

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

stateMat_t& CostFunctionKukaArm_TRK_Contact::getQ()
{
    return Q;
}

stateMat_t& CostFunctionKukaArm_TRK_Contact::getRho_state()
{
    return Rho_state;
}

commandMat_t& CostFunctionKukaArm_TRK_Contact::getRho_torque()
{
    return Rho_torque;
}

stateMat_t& CostFunctionKukaArm_TRK_Contact::getQf()
{
    return Qf;
}

commandMat_t& CostFunctionKukaArm_TRK_Contact::getR()
{
    return R;
}

stateVecTab_t& CostFunctionKukaArm_TRK_Contact::getcx()
{
    return cx_new;
}

commandVecTab_t& CostFunctionKukaArm_TRK_Contact::getcu()
{
    return cu_new;
}

stateMatTab_t& CostFunctionKukaArm_TRK_Contact::getcxx()
{
    return cxx_new;
}

commandR_stateC_tab_t& CostFunctionKukaArm_TRK_Contact::getcux()
{
    return cux_new;
}

commandMatTab_t& CostFunctionKukaArm_TRK_Contact::getcuu()
{
    return cuu_new;
}

double& CostFunctionKukaArm_TRK_Contact::getc()
{
    return c_new;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake