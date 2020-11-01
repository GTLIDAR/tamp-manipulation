#include "drake/traj_gen/admm_contact_constraints/cost_function_contact_new.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

CostFunctionKukaArm_Contact_new::CostFunctionKukaArm_Contact_new(unsigned int N, std::string action_name)
{
    

    // initial, final costs (pos ,vel)
    // torque cost
    // l = sigma(xQx+uRu) + xfQfxf
    if(INCLUDE_OBJECT){
        // num_state = 27; object=7+6; kuka=7+7
        double pos_obj_scale;
        double vel_obj_scale;
        double pos_obj_f_scale;
        double vel_obj_f_scale;
        double torqoe_scale = 30;//100;
        
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

        double torsional_scale = 0;
        double trans_scale = 1e-2;

        QDiagElementVec << pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100, pos_obj_scale*100,
                            vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10, vel_obj_scale*10,
                            pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100, pos_iiwa_scale*100,
                            vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10, vel_iiwa_scale*10;
        QfDiagElementVec << pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0, pos_obj_f_scale*1000.0,
                            vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0, vel_obj_f_scale*100.0,
                            pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0, pos_iiwa_f_scale*1000.0,
                            vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0, vel_iiwa_f_scale*100.0;
        RDiagElementVec << torqoe_scale*0.005, torqoe_scale*0.005, torqoe_scale*0.007, torqoe_scale*0.007, torqoe_scale*0.02, torqoe_scale*0.02, torqoe_scale*0.05;
        FDiagElementVec << torsional_scale, torsional_scale, torsional_scale, trans_scale, trans_scale, trans_scale, 
                           torsional_scale, torsional_scale, torsional_scale, trans_scale, trans_scale, trans_scale; 
    }
    else{
        double pos_scale = 10;
        double vel_scale = 10;
        double pos_f_scale = 1000;//0.001;
        double vel_f_scale = 10;//10;
        double torqoe_scale = 1;//100;

        QDiagElementVec << pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100, pos_scale*100,
                            vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10, vel_scale*10;
        QfDiagElementVec << pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0, pos_f_scale*1000.0,
                            vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0, vel_f_scale*100.0;
        RDiagElementVec << torqoe_scale*0.005, torqoe_scale*0.005, torqoe_scale*0.007, torqoe_scale*0.007, torqoe_scale*0.02, torqoe_scale*0.02, torqoe_scale*0.05;
    }

    Q = QDiagElementVec.asDiagonal();
    Qf = QfDiagElementVec.asDiagonal();
    R = RDiagElementVec.asDiagonal();
    F = FDiagElementVec.asDiagonal();
    // TimeHorizon = total time
    // TimeStep = time between two timesteps
    // N = number of knot
    cx_new.resize(N+1);
    cu_new.resize(N+1);
    cf_new.resize(N+1);
    cxx_new.resize(N+1);
    cux_new.resize(N+1);
    cuu_new.resize(N+1);
    cff_new.resize(N+1);
}

fullstateMat_t& CostFunctionKukaArm_Contact_new::getQ()
{
    return Q;
}

fullstateMat_t& CostFunctionKukaArm_Contact_new::getQf()
{
    return Qf;
}

commandMat_t& CostFunctionKukaArm_Contact_new::getR()
{
    return R;
}

forceMat_t& CostFunctionKukaArm_Contact_new::getF()
{
    return F;
}

fullstateVecTab_t& CostFunctionKukaArm_Contact_new::getcx()
{
    return cx_new;
}

commandVecTab_t& CostFunctionKukaArm_Contact_new::getcu()
{
    return cu_new;
}

forceVecTab_t& CostFunctionKukaArm_Contact_new::getcf()
{
    return cf_new;
}

fullstateMatTab_t& CostFunctionKukaArm_Contact_new::getcxx()
{
    return cxx_new;
}

commandR_fullstateC_tab_t& CostFunctionKukaArm_Contact_new::getcux()
{
    return cux_new;
}

commandMatTab_t& CostFunctionKukaArm_Contact_new::getcuu()
{
    return cuu_new;
}

forceMatTab_t& CostFunctionKukaArm_Contact_new::getcff()
{
    return cff_new;
}

double& CostFunctionKukaArm_Contact_new::getc()
{
    return c_new;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake
