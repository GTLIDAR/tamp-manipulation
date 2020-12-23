#include "drake/traj_gen/admm_contact_constraints/cost_function_contact_track_new.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
	
CostFunctionKukaArm_TRK_Contact_new::CostFunctionKukaArm_TRK_Contact_new(double pos_obj_weight, double pos_iiwa_weight, 
                                                double vel_obj_weight, double vel_iiwa_weight,
                                                double torque_weight, double torsional_force_weight, double trans_force_weight, 
                                                unsigned int N, std::string action_name)
{   
    if(INCLUDE_OBJECT){ 
        // The dimension is 27 in this case
        double pos_obj_scale;
        double vel_obj_scale;
        double pos_obj_f_scale;
        double vel_obj_f_scale;
        double torqoe_scale = 1;//100;
        
        if (action_name.compare("push")==0){
            pos_obj_scale = 0;
            vel_obj_scale = 0;
            pos_obj_f_scale = 0;//0.001;
            vel_obj_f_scale = 0;//10;
        }
        else{
            pos_obj_scale = 0;
            vel_obj_scale = 0;
            pos_obj_f_scale = 0;//0.001;
            vel_obj_f_scale = 0;//10;
        }

        double pos_iiwa_scale = 10;
        double vel_iiwa_scale = 10;
        double pos_iiwa_f_scale = 100;//0.001;
        double vel_iiwa_f_scale = 100;//10;
        
        rho_pos_obj_weight = pos_obj_weight;
        rho_vel_obj_weight = vel_obj_weight;

        rho_pos_iiwa_weight = pos_iiwa_weight;
        rho_vel_iiwa_weight = vel_iiwa_weight;
        rho_torque_weight = torque_weight;
        rho_torsional_force = torsional_force_weight;
        rho_trans_force = trans_force_weight;

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
                            2.5*rho_vel_iiwa_weight, 2.5*rho_vel_iiwa_weight, 1.8*rho_vel_iiwa_weight, 1.25*rho_vel_iiwa_weight, 1.15*rho_vel_iiwa_weight, rho_vel_iiwa_weight, rho_vel_iiwa_weight;
        Rho_torque_DiagElementVec << rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight, rho_torque_weight;
        Rho_force_DiagElementVec << rho_torsional_force, rho_torsional_force, rho_torsional_force, rho_trans_force, rho_trans_force, rho_trans_force,
                                    rho_torsional_force, rho_torsional_force, rho_torsional_force, rho_trans_force, rho_trans_force, rho_trans_force;
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
    Rho_force = Rho_force_DiagElementVec.asDiagonal();

    // TimeHorizon = total time 
    // TimeStep = time between two timesteps
    // N = number of knot
    // N = TimeHorizon/TimeStep;
    cx_new.resize(N+1);
    cu_new.resize(N+1);
    cf_new.resize(N+1);
    cxx_new.resize(N+1);
    cux_new.resize(N+1);
    cuu_new.resize(N+1);
    cff_new.resize(N+1);
}

fullstateMat_t& CostFunctionKukaArm_TRK_Contact_new::getQ()
{
    return Q;
}

fullstateMat_t& CostFunctionKukaArm_TRK_Contact_new::getRho_state()
{
    return Rho_state;
}

commandMat_t& CostFunctionKukaArm_TRK_Contact_new::getRho_torque()
{
    return Rho_torque;
}

forceMat_t& CostFunctionKukaArm_TRK_Contact_new::getRho_force()
{
    return Rho_force;
}

fullstateMat_t& CostFunctionKukaArm_TRK_Contact_new::getQf()
{
    return Qf;
}

commandMat_t& CostFunctionKukaArm_TRK_Contact_new::getR()
{
    return R;
}

fullstateVecTab_t& CostFunctionKukaArm_TRK_Contact_new::getcx()
{
    return cx_new;
}

commandVecTab_t& CostFunctionKukaArm_TRK_Contact_new::getcu()
{
    return cu_new;
}

forceVecTab_t& CostFunctionKukaArm_TRK_Contact_new::getcf()
{
    return cf_new;
}

fullstateMatTab_t& CostFunctionKukaArm_TRK_Contact_new::getcxx()
{
    return cxx_new;
}

commandR_fullstateC_tab_t& CostFunctionKukaArm_TRK_Contact_new::getcux()
{
    return cux_new;
}

commandMatTab_t& CostFunctionKukaArm_TRK_Contact_new::getcuu()
{
    return cuu_new;
}

forceMatTab_t& CostFunctionKukaArm_TRK_Contact_new::getcff()
{
    return cff_new;
}

double& CostFunctionKukaArm_TRK_Contact_new::getc()
{
    return c_new;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake