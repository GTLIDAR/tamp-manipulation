#include "drake/traj_gen/admm_contact_constraints/kuka_arm_contact_new.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

KukaArm_Contact_new::KukaArm_Contact_new(){}

//const char* const kIiwaUrdf =
//    "drake/manipulation/models/iiwa_description/urdf/"
//    "iiwa14_no_collision.urdf";

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";

// const char* const kIiwaUrdf =
//     "drake/manipulation/models/iiwa_description/urdf/"
//     "iiwa7.urdf";

// Add Schunk and Kuka_connector
// const char* const kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
// const char* schunkPath = "drake/manipulation/models/wsg_50_description/urdf/wsg_50_mesh_collision_no_world_joint.urdf";
// const char* connectorPath = "drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf";

// iiwa_dt = time step
// iiwa_N = number of knots
// iiwa_xgoal = final goal in state space (7pos, 7vel)
KukaArm_Contact_new::KukaArm_Contact_new(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal, string action_name)
{
    action_name_ = action_name;
    //#####
    globalcnt = 0;
    //#####
    stateNb = 14;
    commandNb = 7;
    dt = iiwa_dt;
    N = iiwa_N;
    xgoal = iiwa_xgoal;
    fxList.resize(N);
    fuList.resize(N);
    gxList.resize(N);
    guList.resize(N);

    fxxList.resize(fullstateSize);
    for(unsigned int i=0;i<fullstateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;

    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    CC.setZero();
    DD.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    C_temp.resize(N);
    D_temp.resize(N);

    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    if(initial_phase_flag_ == 1){
        // Ye's original method
        initial_phase_flag_ = 0;

        Parser parser(plant_);
        auto iiwa_model = parser.AddModelFromFile(kIiwaUrdf, "iiwa");
        const auto& iiwa_base_frame = plant_->GetFrameByName("iiwa_link_0", iiwa_model);
        RigidTransformd X_WI(Eigen::Vector3d(0, 0, 0));
        plant_->WeldFrames(plant_->world_frame(), iiwa_base_frame, X_WI);

    }
}

KukaArm_Contact_new::KukaArm_Contact_new(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal,
    MultibodyPlant<double>* plant, string action_name)
{
    action_name_ = action_name;
    //#####
    globalcnt = 0;
    //#####
    stateNb = 14;
    commandNb = 7;
    dt = iiwa_dt;
    N = iiwa_N;
    xgoal = iiwa_xgoal;
    fxList.resize(N);
    fuList.resize(N);
    gxList.resize(N);
    guList.resize(N);

    fxxList.resize(fullstateSize);
    for(unsigned int i=0;i<fullstateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;

    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    CC.setZero();
    DD.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    C_temp.resize(N);
    D_temp.resize(N);

    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;


    if(initial_phase_flag_ == 1){
        plant_ = plant;

        initial_phase_flag_ = 0;
    }
}

fullstateVec_t KukaArm_Contact_new::kuka_arm_dynamics(const fullstateVec_t& X, const commandVec_t& tau, forceVec_t& force)
{

    finalTimeProfile.counter0_ += 1;

    if(finalTimeProfile.counter0_ == 10)
        gettimeofday(&tbegin_period,NULL);


    if(INCLUDE_OBJECT){
        if(!DIRECT_INVERSE){
        // object: 7+6; kuka: 7+7 (the joints for wsg cannot be optimized)
        VectorXd q_obj = X.topRows(7);
        Vector4d qua_obj = X.topRows(4); // w, x, y, z
        VectorXd pos_obj = X.middleRows<3>(4);

        VectorXd qd_obj = X.middleRows<6>(7);
        Vector3<double> ang_d_obj = X.middleRows<3>(7);
        VectorXd pos_d_obj = X.middleRows<3>(10);

        VectorXd q_iiwa = X.middleRows<7>(13);
        VectorXd qd_iiwa = X.middleRows<7>(20);

        Eigen::Matrix<double,9,1> q_iiwa_full;
        Eigen::Matrix<double,9,1> qd_iiwa_full;
        VectorXd qd_full(15);
        q_iiwa_full.setZero();
        qd_iiwa_full.setZero();
        q_iiwa_full.topRows(7)=q_iiwa;
        q_iiwa_full.bottomRows(2) << -25, 25;
        qd_iiwa_full.topRows(7)=qd_iiwa;
        qd_full.topRows(6) = qd_obj;
        qd_full.bottomRows(9) = qd_iiwa_full;

        Quaternion<double> qua_obj_eigen(qua_obj(0), qua_obj(1), qua_obj(2), qua_obj(3));

        math::RigidTransform<double> X_WO(qua_obj_eigen, pos_obj);

        auto context_ptr = plant_->CreateDefaultContext();
        auto context = context_ptr.get();
        auto object_model = plant_->GetModelInstanceByName("object");
        auto iiwa_model = plant_->GetModelInstanceByName("iiwa");
        auto wsg_model = plant_->GetModelInstanceByName("wsg");

        plant_->SetFreeBodyPoseInWorldFrame(context, plant_->GetBodyByName("base_link", object_model), X_WO);
        plant_->SetPositions(context, iiwa_model, q_iiwa);
        plant_->SetVelocities(context, iiwa_model, qd_iiwa);
        plant_->SetPositions(context, wsg_model, q_iiwa_full.bottomRows(2));
        plant_->SetVelocities(context, wsg_model, qd_iiwa_full.bottomRows(2));

        // Compute Mass matrix, Bias and gravititional terms
        MatrixXd M_(15, 15);
        VectorXd Cv(15);
        VectorXd tau_g_iiwa;
        VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);
        // bool nan_tau_g_true = false;
        // for (int j = 0; j < tau_g.rows(); j++) {
        //     if (isnan(tau_g(j))) {
        //         std::cout<<"tau_g contains NaN"<<"\n";
        //         // nan_tau_g_true = true;
        //         // std::cout<<Xdot_new.transpose()<<"\n";
        //         break;
        //     }
        // }
        // std::cout<<"tau_g: " << endl << tau_g.transpose()<<"\n";
        // DRAKE_DEMAND(nan_tau_g_true == false);

        plant_->CalcMassMatrix(*context, &M_);
        plant_->CalcBiasTerm(*context, &Cv);
        
        // bool nan_Cv_true = false;
        // for (int j = 0; j < Cv.rows(); j++) {
        //     if (isnan(Cv(j))) {
        //         std::cout<<"Cv contains NaN"<<"\n";
        //         // nan_Cv_true = true;
        //         // std::cout<<Xdot_new.transpose()<<"\n";
        //         break;
        //     }
        // }
        // std::cout<<"Cv: " << endl << Cv.transpose()<<"\n";
        // DRAKE_DEMAND(nan_Cv_true == false);
    

        // Compute Jacobian and AccBias of B_o on finger frame wrt object frame
        const int num_cps = 2;
        Vector3d contact_point_left;
        Vector3d contact_point_right;

        // MatrixXd Jac(3*num_cps, 15);
        // MatrixXd Jac_left(3, 15);
        // MatrixXd Jac_right(3, 15);

        MatrixXd Jac(6*num_cps, 15);
        MatrixXd Jac_left(6, 15);
        MatrixXd Jac_right(6, 15);

        // VectorXd Acc_Bias_left(3);
        // VectorXd Acc_Bias_right(3);

        SpatialAcceleration<double> Acc_Bias_left;
        SpatialAcceleration<double> Acc_Bias_right;
        
        contact_point_left << 0, 0, 0; 
        contact_point_right << 0, 0, 0; 
        // if (action_name_.compare("push")==0){
        //     plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_left); // the second last argument seems doesn't matter?
        //     Acc_Bias_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        //     plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_right); // the second last argument seems doesn't matter?
        //     Acc_Bias_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());
        // }
        // else{
        //     plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_left); // the second last argument seems doesn't matter?
        //     Acc_Bias_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        //     plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_right); // the second last argument seems doesn't matter?
        //     Acc_Bias_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
        //     ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        // }
        // Jac.block<3, 15>(0, 0) = Jac_left;
        // Jac.block<3, 15>(3, 0) = Jac_right;

        if (action_name_.compare("push")==0){
            plant_->CalcJacobianSpatialVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("left_finger", wsg_model), &Jac_left);
            Acc_Bias_left = plant_->CalcBiasSpatialAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("left_finger", wsg_model));

            plant_->CalcJacobianSpatialVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("right_finger", wsg_model), &Jac_right);
            Acc_Bias_right = plant_->CalcBiasSpatialAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("right_finger", wsg_model));
        }
        else{
            plant_->CalcJacobianSpatialVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("left_finger", wsg_model), &Jac_left);
            Acc_Bias_left = plant_->CalcBiasSpatialAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("left_finger", wsg_model));

            plant_->CalcJacobianSpatialVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("right_finger", wsg_model), &Jac_right);
            Acc_Bias_right = plant_->CalcBiasSpatialAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->GetFrameByName("right_finger", wsg_model));

        }

        Jac.block<6, 15>(0, 0) = Jac_left;
        Jac.block<6, 15>(6, 0) = Jac_right;


        // MatrixXd M_Inv = M_.llt().solve(Matrix<double,15,15>::Identity()); 
        // MatrixXd JM_InvJT = Jac * M_Inv * Jac.transpose() + 0 * Matrix<double,3*num_cps,3*num_cps>::Identity();
        // MatrixXd JM_InvJT_Inv = JM_InvJT.llt().solve(Matrix<double,3*num_cps,3*num_cps>::Identity());

        MatrixXd M_Inv = M_.llt().solve(Matrix<double,15,15>::Identity()); 
        MatrixXd JM_InvJT = Jac * M_Inv * Jac.transpose() + 1e-5 * Matrix<double,6*num_cps,6*num_cps>::Identity();
        MatrixXd JM_InvJT_Inv = JM_InvJT.llt().solve(Matrix<double,6*num_cps,6*num_cps>::Identity());
        

        VectorXd Bias_MJ(15);
        // VectorXd Acc_Bias(3*num_cps);
        VectorXd Acc_Bias(6*num_cps);

        Bias_MJ.setZero();
        Bias_MJ = - Cv;
        Bias_MJ.middleRows<6>(0) += tau_g.middleRows<6>(0);
        
        if (action_name_.compare("push")==0){        
            VectorXd dry_friction(6);
            dry_friction << 0, 0, 0, -0.0, 0, 0;
            Bias_MJ.middleRows<6>(0) += dry_friction;
        }
        Bias_MJ.middleRows<7>(6) += tau;

        Acc_Bias.middleRows<6>(0) = -Acc_Bias_left.get_coeffs() - 500*Jac_left*qd_full;
        Acc_Bias.middleRows<6>(6) = -Acc_Bias_right.get_coeffs() - 500*Jac_right*qd_full;
        // bool nan_BiasMJ_true = false;
        // for (int j = 0; j < Bias_MJ.rows(); j++) {
        //     if (isnan(Bias_MJ(j))) {
        //         std::cout<<"Bias_MJ contains NaN"<<"\n";
        //         // nan_BiasMJ_true = true;
        //         // std::cout<<Xdot_new.transpose()<<"\n";
        //         break;
        //     }
        // }
        // std::cout<<"Bias_MJ: " << endl << Bias_MJ.transpose()<<"\n";
        // DRAKE_DEMAND(nan_BiasMJ_true == false);

        //=============================================
        force = JM_InvJT_Inv * (Acc_Bias - Jac * M_Inv*Bias_MJ);
        // cout << force.transpose() << endl;
        VectorXd Acc_total = M_Inv * (Bias_MJ + Jac.transpose() * force);
        VectorXd ang_dd_obj = Acc_total.topRows(3);
        VectorXd pos_dd_obj = Acc_total.middleRows<3>(3);
        if (action_name_.compare("push")==0){
            pos_dd_obj(2,0) = 0.0;
        }
        VectorXd qdd_iiwa = Acc_total.middleRows<7>(6);

        // angular velocity cannot be directly integrated because orientation is not commutive
        VectorXd qua_d_obj = CalculateQuaternionDtFromAngularVelocityExpressedInB(qua_obj_eigen, ang_d_obj);
        Xdot_new << qua_d_obj, pos_d_obj, ang_dd_obj, pos_dd_obj, qd_iiwa, qdd_iiwa;
        
        // bool nan_Xdot_true = false;
        for (int j = 0; j < Xdot_new.rows(); j++) {
            if (isnan(Xdot_new(j))) {
                std::cout<<"New Xdot contains NaN"<<"\n";
                // std::cout<<Xdot_new.transpose()<<"\n";
                break;
            }
        }
        // std::cout<<"Xdot: " << endl << Xdot_new<<"\n";
        // DRAKE_DEMAND(nan_Xdot_true == false);
    

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)
            +((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

        }

    }
    return Xdot_new;
}


KukaArm_Contact_new::timeprofile KukaArm_Contact_new::getFinalTimeProfile()
{
    return finalTimeProfile;
}

void KukaArm_Contact_new::kuka_arm_dyn_cst_ilqr(const fullstateVecTab_t& xList, const commandVecTab_t& uList, const forceVecTab_t& forceList,
                                CostFunctionKukaArm_Contact_new*& costFunction){
    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();

    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();
    CC.setZero();
    DD.setZero();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;

    // const int nargout_update2 = 3;
    for(unsigned int k=0;k<Nl;k++) {
        if(k == Nl-1) {//isNanVec(uList[k])
            // if(debugging_print) TRACE_KUKA_ARM("before the update3\n");
            c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQf()*(xList[k] - xgoal);
            costFunction->getc() += c_mat_to_scalar(0,0);
            // if(debugging_print) TRACE_KUKA_ARM("after the update3\n");
        }
        else {
            // if(debugging_print) TRACE_KUKA_ARM("before the update4\n");
            // FList[k] = update(nargout_update2, xList[k], uList[k], AA, BB);//assume three outputs, code needs to be optimized
            grad(xList[k], uList[k], AA, BB, CC, DD);
            // if(debugging_print) TRACE_KUKA_ARM("before the update4-1\n");
            c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
            // if(debugging_print) TRACE_KUKA_ARM("after the update4\n");

            c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];

            c_mat_to_scalar += 0.5*forceList[k].transpose()*costFunction->getF()*forceList[k];
            costFunction->getc() += c_mat_to_scalar(0,0); // TODO: to be checked
            // if(debugging_print) TRACE_KUKA_ARM("after the update5\n");

            A_temp[k] = AA;
            B_temp[k] = BB;
            C_temp[k] = CC;
            D_temp[k] = DD;
        }
    }

    fullstateVec_t cx_temp;

    if(debugging_print) TRACE_KUKA_ARM("compute dynamics and cost derivative\n");

    for(unsigned int k=0;k<Nl-1;k++){
        fxList[k] = A_temp[k];
        fuList[k] = B_temp[k];
        gxList[k] = C_temp[k];
        guList[k] = D_temp[k];
        // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
        cx_temp << xList[k] - xgoal;

        costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
        costFunction->getcu()[k] = costFunction->getR()*uList[k];
        costFunction->getcf()[k] = costFunction->getF()*forceList[k];
        costFunction->getcxx()[k] = costFunction->getQ();
        costFunction->getcux()[k].setZero();
        costFunction->getcuu()[k] = costFunction->getR();
        costFunction->getcff()[k] = costFunction->getF();
    }
    if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");

    costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
    costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1]; // the final timestep not be used in iLQR 
    costFunction->getcf()[Nl-1] = costFunction->getF()*forceList[Nl-1]; // the final timestep not be used in iLQR
    costFunction->getcxx()[Nl-1] = costFunction->getQf();
    costFunction->getcux()[Nl-1].setZero(); // the final timestep not be used in iLQR
    costFunction->getcuu()[Nl-1] = costFunction->getR(); // the final timestep not be used in iLQR
    costFunction->getcff()[Nl-1] = costFunction->getF(); // the final timestep not be used in iLQR

    if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

    // the following useless matrices are set to Zero.
    //fxx, fxu, fuu are not defined since never used for ilqr
    costFunction->getc() = 0;

    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

void KukaArm_Contact_new::kuka_arm_dyn_cst_min_output(const fullstateVec_t& xList_curr, const commandVec_t& uList_curr, forceVec_t& forceList_curr,
                                                    const bool& isUNan, fullstateVec_t& xList_next, CostFunctionKukaArm_Contact_new*& costFunction){
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");

    costFunction->getc() = 0; // temporary cost container? initializes every timestep

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    xList_next.setZero(); // zeroing previous trajectory timestep by timestep

    if (isUNan) {
        // cout << "R: " <<  costFunction->getR() << endl;
        // cout << "Q: " <<  costFunction->getQ() << endl;
        // cout << "QF: " <<  costFunction->getQf() << endl;
        // if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
        c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose()) * costFunction->getQf() * (xList_curr - xgoal);
        costFunction->getc() += c_mat_to_scalar(0,0);
        // if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
    }
    else {
        // if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
        xList_next = update(xList_curr, uList_curr, forceList_curr);
        c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_curr - xgoal);
        // if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
        c_mat_to_scalar += 0.5*uList_curr.transpose()*costFunction->getR()*uList_curr;
        c_mat_to_scalar += 0.5*forceList_curr.transpose()*costFunction->getF()*forceList_curr;
        costFunction->getc() += c_mat_to_scalar(0,0);
    }

    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

fullstateVec_t KukaArm_Contact_new::update(const fullstateVec_t& X, const commandVec_t& U, forceVec_t& force){
    // 4th-order Runge-Kutta step
    if(debugging_print) TRACE_KUKA_ARM("update: 4th-order Runge-Kutta step\n");

    gettimeofday(&tbegin_period4,NULL);

    // never used
    forceVec_t force1;
    forceVec_t force2;
    forceVec_t force3;

    // output of kuka arm dynamics is xdot = f(x,u)
    Xdot1 = kuka_arm_dynamics(X, U, force);
    Xdot2 = kuka_arm_dynamics(X + 0.5*dt*Xdot1, U, force1);
    Xdot3 = kuka_arm_dynamics(X + 0.5*dt*Xdot2, U, force2);
    Xdot4 = kuka_arm_dynamics(X + dt*Xdot3, U, force3);
    fullstateVec_t X_new;
    X_new = X + (dt/6)*(Xdot1 + 2*Xdot2 + 2*Xdot3 + Xdot4);
    // Simple Euler Integration (for debug)
//    X_new = X + (dt)*Xdot1;

    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    gettimeofday(&tend_period4,NULL);
    finalTimeProfile.time_period4 += (static_cast<double>(1000.0*(tend_period4.tv_sec-tbegin_period4.tv_sec)+((tend_period4.tv_usec-tbegin_period4.tv_usec)/1000.0)))/1000.0;

    return X_new;
}

void KukaArm_Contact_new::grad(const fullstateVec_t& X, const commandVec_t& U, fullstateMat_t& A, fullstateR_commandC_t& B, forceR_fullstateC_t& C, forceR_commandC_t& D){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-7;
    fullstateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    forceVec_t force0;
    forceVec_t force1;
    forceVec_t force2;
    forceVec_t force3;

    for(unsigned int i=0;i<n;i++){
        Xp = update(X+Dx.col(i), U, force0);
        Xm = update(X-Dx.col(i), U, force1);
        A.col(i) = (Xp - Xm)/(2*delta);
        C.col(i) = (force0 - force1)/(2*delta);
    }

    for(unsigned int i=0;i<m;i++){
        Xp = update(X, U+Du.col(i), force2);
        Xm = update(X, U-Du.col(i), force3);
        B.col(i) = (Xp - Xm)/(2*delta);
        D.col(i) = (force2 - force3)/(2*delta);
    }
}

// parameters are called by reference. Name doesn't matter
// void KukaArm_Contact_new::hessian(const fullstateVec_t& X, const commandVec_t& U, fullstateTens_t& fxx_p, fullstateR_fullstateC_commandD_t& fxu_p, fullstateR_commandC_commandD_t& fuu_p){
//     unsigned int n = X.size();
//     unsigned int m = U.size();

//     double delta = 1e-5;
//     fullstateMat_t Dx;
//     commandMat_t Du;
//     Dx.setIdentity();
//     Dx = delta*Dx;
//     Du.setIdentity();
//     Du = delta*Du;

//     fullstateMat_t Ap;
//     Ap.setZero();
//     fullstateMat_t Am;
//     Am.setZero();
//     fullstateR_commandC_t B;
//     B.setZero();

//     for(unsigned int i=0;i<n;i++){
//         fxx_p[i].setZero();
//         fxu_p[i].setZero();
//         fuu_p[i].setZero();
//     }

//     for(unsigned int i=0;i<n;i++){
//         grad(X+Dx.col(i), U, Ap, B);
//         grad(X-Dx.col(i), U, Am, B);
//         fxx_p[i] = (Ap - Am)/(2*delta);
//     }

//     fullstateR_commandC_t Bp;
//     Bp.setZero();
//     fullstateR_commandC_t Bm;
//     Bm.setZero();

//     for(unsigned int j=0;j<m;j++){
//         grad(X, U+Du.col(j), Ap, Bp);
//         grad(X, U-Du.col(j), Am, Bm);
//         fxu_p[j] = (Ap - Am)/(2*delta);
//         fuu_p[j] = (Bp - Bm)/(2*delta);
//     }
// }

unsigned int KukaArm_Contact_new::getStateNb()
{
    return stateNb;
}

unsigned int KukaArm_Contact_new::getCommandNb()
{
    return commandNb;
}

commandVec_t& KukaArm_Contact_new::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& KukaArm_Contact_new::getUpperCommandBounds()
{
    return upperCommandBounds;
}

fullstateMatTab_t& KukaArm_Contact_new::getfxList()
{
    return fxList;
}

fullstateR_commandC_tab_t& KukaArm_Contact_new::getfuList()
{
    return fuList;
}

forceR_fullstateC_tab_t& KukaArm_Contact_new::getgxList()
{
    return gxList;
}

forceR_commandC_tab_t& KukaArm_Contact_new::getguList()
{
    return guList;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake
