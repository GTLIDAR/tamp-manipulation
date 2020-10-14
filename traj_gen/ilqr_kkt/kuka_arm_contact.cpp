#include "drake/traj_gen/ilqr_kkt/kuka_arm_contact.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

KukaArm_Contact::KukaArm_Contact(){}

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
KukaArm_Contact::KukaArm_Contact(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal, string action_name)
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

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
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
    A_temp.resize(N);
    B_temp.resize(N);

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

KukaArm_Contact::KukaArm_Contact(double& iiwa_dt, unsigned int& iiwa_N, fullstateVec_t& iiwa_xgoal,
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

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
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
    A_temp.resize(N);
    B_temp.resize(N);

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

fullstateVec_t KukaArm_Contact::kuka_arm_dynamics(const fullstateVec_t& X, const commandVec_t& tau)
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
        VectorXd qd_iiwa = X.bottomRows(7);

        Eigen::Matrix<double,9,1> q_iiwa_full;
        Eigen::Matrix<double,9,1> qd_iiwa_full;
        VectorXd qd_full(15);
        q_iiwa_full.setZero();
        qd_iiwa_full.setZero();
        q_iiwa_full.topRows(7)=q_iiwa;
        q_iiwa_full.bottomRows(2) << -0.025, 0.025;
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
        // MatrixXd M_iiwa(7, 7);
        VectorXd Cv(15);
        // VectorXd Cv_iiwa(7);
        VectorXd tau_g_iiwa;
        VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);
        // bool nan_tau_g_true = false;
        for (int j = 0; j < tau_g.rows(); j++) {
            if (isnan(tau_g(j))) {
                std::cout<<"tau_g contains NaN"<<"\n";
                // nan_tau_g_true = true;
                // std::cout<<Xdot_new.transpose()<<"\n";
                break;
            }
        }
        // std::cout<<"tau_g: " << endl << tau_g.transpose()<<"\n";
        // DRAKE_DEMAND(nan_tau_g_true == false);

        plant_->CalcMassMatrix(*context, &M_);
        plant_->CalcBiasTerm(*context, &Cv);
        
        // bool nan_Cv_true = false;
        for (int j = 0; j < Cv.rows(); j++) {
            if (isnan(Cv(j))) {
                std::cout<<"Cv contains NaN"<<"\n";
                // nan_Cv_true = true;
                // std::cout<<Xdot_new.transpose()<<"\n";
                break;
            }
        }
        // std::cout<<"Cv: " << endl << Cv.transpose()<<"\n";
        // DRAKE_DEMAND(nan_Cv_true == false);
        
        // M_iiwa = M_.block<7, 7>(6, 6);
        // Cv_iiwa = Cv.middleRows<7>(6);
        // tau_g_iiwa = tau_g.middleRows<7>(6);

        // Compute Jacobian and AccBias of B_o on finger frame wrt object frame
        const int num_cps = 2;
        Vector3d contact_point_left;
        Vector3d contact_point_right;

        MatrixXd Jac(3*num_cps, 15);
        MatrixXd Jac_left(3, 15);
        MatrixXd Jac_right(3, 15);
        // MatrixXd Jac_down_left(3, 15);
        // MatrixXd Jac_down_right(3, 15);

        Vector3d Acc_Bias_left;
        Vector3d Acc_Bias_right;
        // Vector3d Acc_Bias_down_left;
        // Vector3d Acc_Bias_down_right;
        
        contact_point_left << 0, 0, 0; 
        contact_point_right << 0, 0, 0; 
        if (action_name_.compare("push")==0){
            plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_left); // the second last argument seems doesn't matter?
            Acc_Bias_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact3", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

            plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_right); // the second last argument seems doesn't matter?
            Acc_Bias_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact3", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());
        }
        else{
            plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_left); // the second last argument seems doesn't matter?
            Acc_Bias_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

            plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_right); // the second last argument seems doesn't matter?
            Acc_Bias_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

            // plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_down_left); // the second last argument seems doesn't matter?
            // Acc_Bias_down_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
            // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

            // plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_down_right); // the second last argument seems doesn't matter?
            // Acc_Bias_down_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
            // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());
        }
        Jac.block<3, 15>(0, 0) = Jac_left;
        Jac.block<3, 15>(3, 0) = Jac_right;
        // Jac.block<3, 15>(6, 0) = Jac_down_left;
        // Jac.block<3, 15>(9, 0) = Jac_down_right;

        MatrixXd M_Inv = M_.llt().solve(Matrix<double,15,15>::Identity()); 
        MatrixXd JM_InvJT = Jac * M_Inv * Jac.transpose() + 0 * Matrix<double,3*num_cps,3*num_cps>::Identity();
        MatrixXd JM_InvJT_Inv = JM_InvJT.llt().solve(Matrix<double,3*num_cps,3*num_cps>::Identity());
        

        VectorXd Bias_MJ(15);
        VectorXd Acc_Bias(3*num_cps);

        Bias_MJ.setZero();
        Bias_MJ = - Cv;
        Bias_MJ.middleRows<6>(0) += tau_g.middleRows<6>(0);
        
        if (action_name_.compare("push")==0){        
            VectorXd dry_friction(6);
            dry_friction << 0, 0, 0, -0.0, 0, 0;
            Bias_MJ.middleRows<6>(0) += dry_friction;
        }
        Bias_MJ.middleRows<7>(6) += tau;

        Acc_Bias.middleRows<3>(0) = -Acc_Bias_left - 500*Jac_left*qd_full;
        Acc_Bias.middleRows<3>(3) = -Acc_Bias_right - 500*Jac_right*qd_full;
        // Acc_Bias.middleRows<3>(6) = -Acc_Bias_down_left - 500*Jac_down_left*qd_full;
        // Acc_Bias.middleRows<3>(9) = -Acc_Bias_down_right - 500*Jac_down_right*qd_full;
        // bool nan_BiasMJ_true = false;
        for (int j = 0; j < Bias_MJ.rows(); j++) {
            if (isnan(Bias_MJ(j))) {
                std::cout<<"Bias_MJ contains NaN"<<"\n";
                // nan_BiasMJ_true = true;
                // std::cout<<Xdot_new.transpose()<<"\n";
                break;
            }
        }
        // std::cout<<"Bias_MJ: " << endl << Bias_MJ.transpose()<<"\n";
        // DRAKE_DEMAND(nan_BiasMJ_true == false);

        //=============================================
        // vd = M_iiwa.inverse()*(tau - Cv_iiwa + Jac_iiwa.transpose() * f_ext);

        // MatrixXd M_JInv = M_J.inverse();
        // VectorXd Acc_total = M_JInv*Bias_MJ;
        VectorXd force = JM_InvJT_Inv * (Acc_Bias - Jac * M_Inv*Bias_MJ);
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
        

        // if(nan_true){
        //     cout << "Bias_MJ: " << Bias_MJ.transpose() << endl;
        //     cout << "Matrix M_J: " << M_J << endl;
        // }

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)
            +((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

        }

        ////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////
        else{
         // object: 7+6; kuka: 7+7 (the joints for wsg cannot be optimized)
        VectorXd q_obj = X.topRows(7);
        Vector4d qua_obj = X.topRows(4); // w, x, y, z
        VectorXd pos_obj = X.middleRows<3>(4);

        VectorXd qd_obj = X.middleRows<6>(7);
        Vector3<double> ang_d_obj = X.middleRows<3>(7);
        VectorXd pos_d_obj = X.middleRows<3>(10);

        VectorXd q_iiwa = X.middleRows<7>(13);
        VectorXd qd_iiwa = X.bottomRows(7);

        Eigen::Matrix<double,9,1> q_iiwa_full;
        Eigen::Matrix<double,9,1> qd_iiwa_full;
        VectorXd qd_full(15);
        q_iiwa_full.setZero();
        qd_iiwa_full.setZero();
        q_iiwa_full.topRows(7)=q_iiwa;
        q_iiwa_full.bottomRows(2) << -0.025, 0.025;
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
        // MatrixXd M_iiwa(7, 7);
        VectorXd Cv(15);
        // VectorXd Cv_iiwa(7);
        VectorXd tau_g_iiwa;
        VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);

        plant_->CalcMassMatrix(*context, &M_);
        plant_->CalcBiasTerm(*context, &Cv);
        bool nan_Cv_true = false;
        for (int j = 0; j < Cv.rows(); j++) {
            if (isnan(Cv(j))) {
                std::cout<<"Cv contains NaN"<<"\n";
                nan_Cv_true = true;
                // std::cout<<Xdot_new.transpose()<<"\n";
                break;
            }
        }
        // std::cout<<"Cv: " << endl << Cv.transpose()<<"\n";
        DRAKE_DEMAND(nan_Cv_true == false);
        // M_iiwa = M_.block<7, 7>(6, 6);
        // Cv_iiwa = Cv.middleRows<7>(6);
        // tau_g_iiwa = tau_g.middleRows<7>(6);

        // Compute Jacobian and AccBias of B_o on finger frame wrt object frame
        const int num_cps = 2;
        Vector3d contact_point_left;
        Vector3d contact_point_right;
        // Vector3d contact_point_left2;
        // Vector3d contact_point_right2;
        MatrixXd Jac(3*num_cps, 15);
        MatrixXd Jac_left(3, 15);
        MatrixXd Jac_right(3, 15);
        // MatrixXd Jac_down_left(3, 15);
        // MatrixXd Jac_down_right(3, 15);

        Vector3d Acc_Bias_left;
        Vector3d Acc_Bias_right;
        // Vector3d Acc_Bias_down_left;
        // Vector3d Acc_Bias_down_right;
        
        contact_point_left << 0, 0, 0; 
        contact_point_right << 0, 0, 0; 
        plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact2", wsg_model), contact_point_left
        ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_left); // the second last argument seems doesn't matter?
        Acc_Bias_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact2", wsg_model), contact_point_left
        ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact2", wsg_model), contact_point_right
        ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_right); // the second last argument seems doesn't matter?
        Acc_Bias_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact2", wsg_model), contact_point_right
        ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        // plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
        // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_down_left); // the second last argument seems doesn't matter?
        // Acc_Bias_down_left = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("left_ball_contact1", wsg_model), contact_point_left
        // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        // plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
        // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame(), &Jac_down_right); // the second last argument seems doesn't matter?
        // Acc_Bias_down_right = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("right_ball_contact1", wsg_model), contact_point_right
        // ,plant_->GetFrameByName("base_link", object_model), plant_->world_frame());

        Jac.block<3, 15>(0, 0) = Jac_left;
        Jac.block<3, 15>(3, 0) = Jac_right;
        // Jac.block<3, 15>(6, 0) = Jac_down_left;
        // Jac.block<3, 15>(9, 0) = Jac_down_right;

        MatrixXd M_J;
        M_J.setZero(15+3*num_cps, 15+3*num_cps);
        M_J.block<15, 15>(0, 0) = M_;
        M_J.block<15, 3*num_cps>(0, 15) = Jac.transpose();
        M_J.block<3*num_cps, 15>(15, 0) = Jac;
        // M_J += 1e-2 * Matrix<double,15+3*num_cps,15+3*num_cps>::Identity();
        
        VectorXd Bias_MJ(15+3*num_cps);
        Bias_MJ.setZero();
        Bias_MJ.topRows(15) = - Cv;
        Bias_MJ.middleRows<6>(0) += tau_g.middleRows<6>(0);
        Bias_MJ.middleRows<7>(6) += tau;
        Bias_MJ.middleRows<3>(15) = -Acc_Bias_left - 500*Jac_left*qd_full;
        Bias_MJ.middleRows<3>(18) = -Acc_Bias_right - 500*Jac_right*qd_full;
        // Bias_MJ.middleRows<3>(21) = -Acc_Bias_down_left - 500*Jac_down_left*qd_full;
        // Bias_MJ.middleRows<3>(24) = -Acc_Bias_down_right - 500*Jac_down_right*qd_full;

        //=============================================
        // vd = M_iiwa.inverse()*(tau - Cv_iiwa + Jac_iiwa.transpose() * f_ext);
        MatrixXd M_JInv = M_J.inverse();
        VectorXd Acc_total = M_JInv*Bias_MJ;
        VectorXd ang_dd_obj = Acc_total.topRows(3);
        VectorXd pos_dd_obj = Acc_total.middleRows<3>(3);
        VectorXd qdd_iiwa = Acc_total.middleRows<7>(6);

        bool nan_MJInv_true = false;
        for(int i = 0; i < M_JInv.rows(); i++){
            for (int j = 0; j < M_JInv.cols(); j++) {
                if (isnan(M_JInv(i,j))) {
                    std::cout<<"M_J's inverse contains NaN"<<"\n";
                    // std::cout<<transpose()<<"\n";
                    nan_MJInv_true = true;
                    break;
                }

            }
            if(nan_MJInv_true){break;}
        }
        DRAKE_DEMAND(nan_MJInv_true == false);

        // angular velocity cannot be directly integrated because orientation is not commutive
        VectorXd qua_d_obj = CalculateQuaternionDtFromAngularVelocityExpressedInB(qua_obj_eigen, ang_d_obj);
        Xdot_new << qua_d_obj, pos_d_obj, ang_dd_obj, pos_dd_obj, qd_iiwa, qdd_iiwa;
        
        bool nan_Xdot_true = false;
        for (int j = 0; j < Xdot_new.rows(); j++) {
            if (isnan(Xdot_new(j))) {
                std::cout<<"New Xdot contains NaN"<<"\n";
                // std::cout<<Xdot_new.transpose()<<"\n";
                nan_Xdot_true = true;
                break;
            }
        }
        // std::cout<<"Xdot: " << endl << Xdot_new<<"\n";
        DRAKE_DEMAND(nan_Xdot_true == false);

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)
            +((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

        }
    }
    // SHOULD ALWAYS HAVE OBJECT DYNAMICS! THIS IS JUST FOR TESTING THE STATIC CONTACT POINT
    else{
        q << X.head(stateSize/2);
        qd << X.tail(stateSize/2);

        Eigen::Matrix<double,stateSize/2+2,1> q_full;
        Eigen::Matrix<double,stateSize/2+2,1> qd_full;
        q_full.setZero();
        qd_full.setZero();
        q_full.topRows(stateSize/2)=q;
        qd_full.topRows(stateSize/2)=qd;

        auto rpy = math::RollPitchYawd(Eigen::Vector3d(0, 0, 0));

        auto xyz = Eigen::Vector3d(1, 1, 1);

        math::RigidTransform<double> X_WO(math::RotationMatrix<double>(rpy), xyz);

        auto context_ptr = plant_->CreateDefaultContext();
        auto context = context_ptr.get();
        auto object_model = plant_->GetModelInstanceByName("object");
        auto iiwa_model = plant_->GetModelInstanceByName("iiwa");
        // auto wsg_model = plant_->GetModelInstanceByName("wsg");

        plant_->SetFreeBodyPoseInWorldFrame(context, plant_->GetBodyByName("base_link_cracker", object_model), X_WO);
        plant_->SetPositions(context, iiwa_model, q);
        plant_->SetVelocities(context, iiwa_model, qd);

        // Compute Mass matrix, Bias and gravititional terms
        MatrixXd M_(15, 15);
        MatrixXd M_iiwa(7, 7);
        VectorXd Cv(15);
        VectorXd Cv_iiwa(7);
        VectorXd tau_g_iiwa;
        VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);

        plant_->CalcMassMatrix(*context, &M_);
        plant_->CalcBiasTerm(*context, &Cv);
        M_iiwa = M_.block<7, 7>(6, 6);
        Cv_iiwa = Cv.middleRows<7>(6);
        tau_g_iiwa = tau_g.middleRows<7>(6);

        // Compute Jacobian and AccBias of B_o on finger frame wrt object frame
        Vector3d contact_point;
        MatrixXd Jac(3, 15);
        MatrixXd Jac_iiwa(3, 7);
        Vector3d Acc_Bias;

        contact_point << 0, 0, 0; 
        plant_->CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("iiwa_frame_ee", iiwa_model), contact_point
        ,plant_->GetFrameByName("base_link_cracker", object_model), plant_->world_frame(), &Jac);
        Acc_Bias = plant_->CalcBiasTranslationalAcceleration(*context, JacobianWrtVariable::kV, plant_->GetFrameByName("iiwa_frame_ee", iiwa_model), contact_point
        ,plant_->GetFrameByName("base_link_cracker", object_model), plant_->world_frame());
        Jac_iiwa = Jac.middleCols(6, 7);

        MatrixXd M_J;
        M_J.setZero(10, 10);
        M_J.block<7, 7>(0, 0) = M_iiwa;
        M_J.block<7, 3>(0, 7) = Jac_iiwa.transpose();
        M_J.block<3, 7>(7, 0) = Jac_iiwa;
        
        VectorXd Bias_MJ(10);
        Bias_MJ.setZero();
        Bias_MJ.topRows(7) = tau + tau_g_iiwa - Cv_iiwa;
        Bias_MJ.bottomRows(3) = -Acc_Bias;
        
        cout << "Cv_iiwa: " << Cv_iiwa.transpose() << endl;
        cout << "tau_g_iiwa: " << tau_g_iiwa.transpose() << endl;
        cout << "tau: " << tau.transpose() << endl;
        // VectorXd bias_term_ = plant_->CalcGravityGeneralizedForces(*context); // Gravity Comp        
        //=============================================
        // vd = M_iiwa.inverse()*(tau - Cv_iiwa + Jac_iiwa.transpose() * f_ext);
        VectorXd Acc_total = M_J.inverse()*Bias_MJ;
        vd = Acc_total.topRows(7);

        Xdot_new << qd, vd;

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)
            +((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

        // vdot is newly calculated using q, qdot, u
        // (qdot, vdot) = f((q, qdot), u) ??? Makes sense??
    }
    return Xdot_new;
}


KukaArm_Contact::timeprofile KukaArm_Contact::getFinalTimeProfile()
{
    return finalTimeProfile;
}

void KukaArm_Contact::kuka_arm_dyn_cst_ilqr(const int& nargout, const fullstateVecTab_t& xList, const commandVecTab_t& uList, fullstateVecTab_t& FList,
                                CostFunctionKukaArm_Contact*& costFunction){
    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();

    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;

    if(nargout == 2){
        const int nargout_update1 = 3;
        for(unsigned int k=0;k<Nl;k++){
            if(k == Nl-1){//isNanVec(uList[k])
                if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
            }else{
                // if u is not NaN (not final), add state and control cost
                if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }
    else {
        const int nargout_update2 = 3;
        for(unsigned int k=0;k<Nl;k++) {
            if(k == Nl-1) {//isNanVec(uList[k])
                // if(debugging_print) TRACE_KUKA_ARM("before the update3\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQf()*(xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                // if(debugging_print) TRACE_KUKA_ARM("after the update3\n");
            }
            else {
                // if(debugging_print) TRACE_KUKA_ARM("before the update4\n");
                FList[k] = update(nargout_update2, xList[k], uList[k], AA, BB);//assume three outputs, code needs to be optimized
                // if(debugging_print) TRACE_KUKA_ARM("before the update4-1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                // if(debugging_print) TRACE_KUKA_ARM("after the update4\n");

                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0); // TODO: to be checked
                // if(debugging_print) TRACE_KUKA_ARM("after the update5\n");

                A_temp[k] = AA;
                B_temp[k] = BB;
            }
        }

        fullstateVec_t cx_temp;

        if(debugging_print) TRACE_KUKA_ARM("compute dynamics and cost derivative\n");

        for(unsigned int k=0;k<Nl-1;k++){
            fxList[k] = A_temp[k];
            fuList[k] = B_temp[k];

            // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            cx_temp << xList[k] - xgoal;

            costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            costFunction->getcu()[k] = costFunction->getR()*uList[k];
            costFunction->getcxx()[k] = costFunction->getQ();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR();
        }
        if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");

        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        costFunction->getcxx()[Nl-1] = costFunction->getQf();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR();

        if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

        // the following useless matrices are set to Zero.
        //fxx, fxu, fuu are not defined since never used
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

void KukaArm_Contact::kuka_arm_dyn_cst_min_output(const int& nargout, const fullstateVec_t& xList_curr, const commandVec_t& uList_curr, 
                                                    const bool& isUNan, fullstateVec_t& xList_next, CostFunctionKukaArm_Contact*& costFunction){
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    if(debugging_print) std::cout<<"nargout: "<<nargout<<"\n";
    unsigned int Nc = xList_curr.cols(); //xList_curr is 14x1 vector -> col=1

    costFunction->getc() = 0; // temporary cost container? initializes every timestep
    AA.setZero();
    BB.setZero();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    xList_next.setZero(); // zeroing previous trajectory timestep by timestep

    const int nargout_update1 = 1;
    for(unsigned int k=0;k<Nc;k++) {
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
            xList_next = update(nargout_update1, xList_curr, uList_curr, AA, BB);
            c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_curr - xgoal);
            // if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
            c_mat_to_scalar += 0.5*uList_curr.transpose()*costFunction->getR()*uList_curr;
            costFunction->getc() += c_mat_to_scalar(0,0);
        }
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

fullstateVec_t KukaArm_Contact::update(const int& nargout, const fullstateVec_t& X, const commandVec_t& U, fullstateMat_t& A, fullstateR_commandC_t& B){
    // 4th-order Runge-Kutta step
    if(debugging_print) TRACE_KUKA_ARM("update: 4th-order Runge-Kutta step\n");

    gettimeofday(&tbegin_period4,NULL);

    // output of kuka arm dynamics is xdot = f(x,u)
    Xdot1 = kuka_arm_dynamics(X, U);
    Xdot2 = kuka_arm_dynamics(X + 0.5*dt*Xdot1, U);
    Xdot3 = kuka_arm_dynamics(X + 0.5*dt*Xdot2, U);
    Xdot4 = kuka_arm_dynamics(X + dt*Xdot3, U);
    fullstateVec_t X_new;
    X_new = X + (dt/6)*(Xdot1 + 2*Xdot2 + 2*Xdot3 + Xdot4);
    // Simple Euler Integration (for debug)
//    X_new = X + (dt)*Xdot1;

    if ((globalcnt%4 == 0) && (globalcnt<40)) {
        // cout << "X " << endl << X << endl;
        // cout << "Xdot1 " << endl << Xdot1 << endl;
        // cout << "Xdot2 " << endl << Xdot2 << endl;
        // cout << "Xdot3 " << endl << Xdot3 << endl;
        // cout << "Xdot4 " << endl << Xdot4 << endl;
        // cout << "X_NEW: " << endl << X_new << endl;
    }

    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    //######3
    // int as = 0;
    //#########3

    if(nargout > 1){// && (as!=0)){
        //cout << "NEVER HERE" << endl;
        unsigned int n = X.size();
        unsigned int m = U.size();

        double delta = 1e-7;
        fullstateMat_t Dx;
        commandMat_t Du;
        Dx.setIdentity();
        Dx = delta*Dx;
        Du.setIdentity();
        Du = delta*Du;

        // State perturbation
        for(unsigned int i=0;i<n;i++){
            Xp1 = kuka_arm_dynamics(X+Dx.col(i),U);
            Xm1 = kuka_arm_dynamics(X-Dx.col(i),U);
            A1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = kuka_arm_dynamics(X+0.5*dt*Xdot1+Dx.col(i),U);
            Xm2 = kuka_arm_dynamics(X+0.5*dt*Xdot1-Dx.col(i),U);
            A2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = kuka_arm_dynamics(X+0.5*dt*Xdot2+Dx.col(i),U);
            Xm3 = kuka_arm_dynamics(X+0.5*dt*Xdot2-Dx.col(i),U);
            A3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = kuka_arm_dynamics(X+dt*Xdot3+Dx.col(i),U);
            Xm4 = kuka_arm_dynamics(X+dt*Xdot3-Dx.col(i),U);
            A4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        // Control perturbation
        for(unsigned int i=0;i<m;i++){
            Xp1 = kuka_arm_dynamics(X,U+Du.col(i));
            Xm1 = kuka_arm_dynamics(X,U-Du.col(i));
            B1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = kuka_arm_dynamics(X+0.5*dt*Xdot1,U+Du.col(i));
            Xm2 = kuka_arm_dynamics(X+0.5*dt*Xdot1,U-Du.col(i));
            B2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = kuka_arm_dynamics(X+0.5*dt*Xdot2,U+Du.col(i));
            Xm3 = kuka_arm_dynamics(X+0.5*dt*Xdot2,U-Du.col(i));
            B3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = kuka_arm_dynamics(X+dt*Xdot3,U+Du.col(i));
            Xm4 = kuka_arm_dynamics(X+dt*Xdot3,U-Du.col(i));
            B4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        A = (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)*(IdentityMat + A2 * dt/3)*(IdentityMat + A1 * dt/6);
        B = B4 * dt/6 + (IdentityMat + A4 * dt/6) * B3 * dt/3 + (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)* B2 * dt/3 
            + (IdentityMat + (dt/6)*A4)*(IdentityMat + (dt/3)*A3)*(IdentityMat + (dt/3)*A2)*(dt/6)*B1;
    }
    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    gettimeofday(&tend_period4,NULL);
    finalTimeProfile.time_period4 += (static_cast<double>(1000.0*(tend_period4.tv_sec-tbegin_period4.tv_sec)+((tend_period4.tv_usec-tbegin_period4.tv_usec)/1000.0)))/1000.0;

    return X_new;
}

void KukaArm_Contact::grad(const fullstateVec_t& X, const commandVec_t& U, fullstateMat_t& A, fullstateR_commandC_t& B){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-7;
    fullstateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    AA.setZero();
    BB.setZero();

    int nargout = 1;
    for(unsigned int i=0;i<n;i++){
        Xp = update(nargout, X+Dx.col(i), U, AA, BB);
        Xm = update(nargout, X-Dx.col(i), U, AA, BB);
        A.col(i) = (Xp - Xm)/(2*delta);
    }

    for(unsigned int i=0;i<m;i++){
        Xp = update(nargout, X, U+Du.col(i), AA, BB);
        Xm = update(nargout, X, U-Du.col(i), AA, BB);
        B.col(i) = (Xp - Xm)/(2*delta);
    }
}

// parameters are called by reference. Name doesn't matter
void KukaArm_Contact::hessian(const fullstateVec_t& X, const commandVec_t& U, fullstateTens_t& fxx_p, fullstateR_fullstateC_commandD_t& fxu_p, fullstateR_commandC_commandD_t& fuu_p){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-5;
    fullstateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    fullstateMat_t Ap;
    Ap.setZero();
    fullstateMat_t Am;
    Am.setZero();
    fullstateR_commandC_t B;
    B.setZero();

    for(unsigned int i=0;i<n;i++){
        fxx_p[i].setZero();
        fxu_p[i].setZero();
        fuu_p[i].setZero();
    }

    for(unsigned int i=0;i<n;i++){
        grad(X+Dx.col(i), U, Ap, B);
        grad(X-Dx.col(i), U, Am, B);
        fxx_p[i] = (Ap - Am)/(2*delta);
    }

    fullstateR_commandC_t Bp;
    Bp.setZero();
    fullstateR_commandC_t Bm;
    Bm.setZero();

    for(unsigned int j=0;j<m;j++){
        grad(X, U+Du.col(j), Ap, Bp);
        grad(X, U-Du.col(j), Am, Bm);
        fxu_p[j] = (Ap - Am)/(2*delta);
        fuu_p[j] = (Bp - Bm)/(2*delta);
    }
}

unsigned int KukaArm_Contact::getStateNb()
{
    return stateNb;
}

unsigned int KukaArm_Contact::getCommandNb()
{
    return commandNb;
}

commandVec_t& KukaArm_Contact::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& KukaArm_Contact::getUpperCommandBounds()
{
    return upperCommandBounds;
}

fullstateMatTab_t& KukaArm_Contact::getfxList()
{
    return fxList;
}

fullstateR_commandC_tab_t& KukaArm_Contact::getfuList()
{
    return fuList;
}


void KukaArm_Contact::kuka_arm_dyn_cst_udp(const int& nargout, const fullstateVecTab_t& xList, const commandVecTab_t& uList, fullstateVecTab_t& FList,
                                CostFunctionKukaArm_Contact*& costFunction){
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();

    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();
    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    c_mat_to_scalar.setZero();

    if(nargout == 2){
        const int nargout_update1 = 3;
        for(unsigned int k=0;k<Nl;k++){
            if (k == Nl-1){
                if(debugging_print) TRACE_KUKA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
            }else{
                if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }else{
        fullstateVec_t cx_temp;
        if(debugging_print) TRACE_KUKA_ARM("compute cost derivative\n");
        for(unsigned int k=0;k<Nl-1;k++){
            // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            cx_temp << xList[k] - xgoal;

            costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            costFunction->getcu()[k] = costFunction->getR()*uList[k];
            costFunction->getcxx()[k] = costFunction->getQ();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR();
        }
        if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");
        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        costFunction->getcxx()[Nl-1] = costFunction->getQf();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR();
        if(debugging_print) TRACE_KUKA_ARM("set unused matrices to zero \n");

        // the following useless matrices and scalars are set to Zero.
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake
