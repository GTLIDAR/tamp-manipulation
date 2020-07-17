#include "drake/traj_gen/kuka_arm_track.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

KukaArm_TRK::KukaArm_TRK(){}

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
KukaArm_TRK::KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, string action_name)
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

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
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

KukaArm_TRK::KukaArm_TRK(double& iiwa_dt, unsigned int& iiwa_N, stateVec_t& iiwa_xgoal, MultibodyPlant<double>* plant, string action_name)
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

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
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

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    if(initial_phase_flag_ == 1){
        plant_ = plant;

        initial_phase_flag_ = 0;
    }
}

stateVec_t KukaArm_TRK::kuka_arm_dynamics(const stateVec_t& X, const commandVec_t& tau)
{

    finalTimeProfile.counter0_ += 1;

    if(finalTimeProfile.counter0_ == 10)
        gettimeofday(&tbegin_period,NULL);

    q << X.head(stateSize/2);
    qd << X.tail(stateSize/2);

    if(WHOLE_BODY){
        Eigen::Matrix<double,stateSize/2+2,1> q_full;
        Eigen::Matrix<double,stateSize/2+2,1> qd_full;
        q_full.setZero();
        qd_full.setZero();
        q_full.topRows(stateSize/2)=q;
        qd_full.topRows(stateSize/2)=qd;

        auto context_ptr = plant_->CreateDefaultContext();
        auto context = context_ptr.get();
        plant_->SetPositions(context, q_full);
        plant_->SetVelocities(context, qd_full);

        MatrixXd M_(plant_->num_velocities(), plant_->num_velocities());
        plant_->CalcMassMatrix(*context, &M_);

        //===================
        // External Force
        //==================

        // TODO: Add external force for MBP
        // if (action_name_.compare("push")==0) {
        // // 1) Pushing External Force
        //     const auto &body_pose_fk = robot_thread_->CalcFramePoseInWorldFrame(
        //                             cache_, robot_thread_->get_body(10), Isometry3d::Identity());
        //     // cout << "forward kinematics init\n" << body_pose_fk.linear() << endl;
        //     // cout << "forward kinematics init\n" << body_pose_fk.translation() << endl << endl;
        //     if (body_pose_fk.translation().x() > 0.088 ) { // {pushing: 0.087 ; sliding: 0.186; throwing: }
        //         drake::WrenchVector<double> tw;
        //         tw  <<  0,0,0,-0.6*BOXWEIGHT*9.81,0,0; // pushing (rx, ry, rz, tx, ty, tz)
        //         // tw  <<  0,0,0,0,-0.6*BOXWEIGHT*9.81,0; // sliding
        //         RigidBody<double> const & rb = (*robot_thread_).get_body(10);
        //         f_ext[&rb] = tw;
        //     }
        // } else if (action_name_.compare("throw")==0) {

        // // 2) Throwing External Force
        //     const auto &body_pose_fk = robot_thread_->CalcFramePoseInWorldFrame(
        //                             cache_, robot_thread_->get_body(10), Isometry3d::Identity());
        //     math::RotationMatrix<double> R_init(body_pose_fk.linear());
        //     math::RollPitchYaw<double> rpy_init(R_init);
        //     if (body_pose_fk.translation().x() < 0.720779) {        // {pushing: 0.087 ; sliding: 0.186; throwing: 0.720779}
        //         drake::WrenchVector<double> tw;
        //         tw  <<  0,0,0,0,0,-BOXWEIGHT*9.81*sin(-rpy_init.pitch_angle());
        //         RigidBody<double> const & rb = (*robot_thread_).get_body(10);
        //         f_ext[&rb] = tw;
        //     }
        // }

        //========================================


        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period3 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

        //gettimeofday(&tbegin_period,NULL);

        // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda

        // MultibodyForces<double> f_ext(*plant_);
        // VectorXd vdot(plant_->num_velocities());
        // vdot.setZero();
        // VectorXd bias_term_ = plant_->CalcInverseDynamics(*context, vdot, f_ext);
        VectorXd bias_term_ = plant_->CalcGravityGeneralizedForces(*context);

        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period4 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

        //=============================================
        // Gravity compensation?? - From Yuki

        //Set false for doing only gravity comp
        //     VectorX<double> gtau = robot_thread_->inverseDynamics(cache_, f_ext, qd_0, false);
        //=============================================
        vd = (M_.inverse()*(tau - bias_term_)).head(stateSize/2);
        //    vd = M_.inverse()*(tau + bias_term_2 - bias_term_ );
        //    vd = M_.inverse()*(tau - bias_term_ + gtau);
        //    vd = M_.inverse()*(tau + gtau);
        Xdot_new << qd, vd;

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

    }
    else{
        auto context_ptr = plant_->CreateDefaultContext();
        auto context = context_ptr.get();
        plant_->SetPositions(context, q);
        plant_->SetVelocities(context, qd);

        MatrixXd M_;
        plant_->CalcMassMatrix(*context, &M_);

        VectorXd bias_term_ = plant_->CalcGravityGeneralizedForces(*context);

        vd = (M_.inverse()*(tau - bias_term_));
        Xdot_new << qd, vd;

        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }

        if (globalcnt < 40)
            globalcnt += 1;

    }
    return Xdot_new;
}


KukaArm_TRK::timeprofile KukaArm_TRK::getFinalTimeProfile()
{
    return finalTimeProfile;
}

void KukaArm_TRK::kuka_arm_dyn_cst_ilqr(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList,
                                const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, CostFunctionKukaArm_TRK*& costFunction){
    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    // for ADMM, add new penalties for augmented Lagrangian terms
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
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal) +
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);
                // cout << "CMAT SCALAR 1: " << c_mat_to_scalar << endl;
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
            }else{
                // if u is not NaN (not final), add state and control cost
                if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal) +
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);

                // cout << "CMAT SCALAR 2: " << c_mat_to_scalar << endl;

                if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k] +
                0.5*(uList[k].transpose() - uList_bar[k].transpose()) * costFunction->getRho_torque()* (uList[k] - uList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }
    else {
        const int nargout_update2 = 3;
        for(unsigned int k=0;k<Nl;k++) {
            if(k == Nl-1) {//isNanVec(uList[k])
                // if(debugging_print) TRACE_KUKA_ARM("before the update3\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQf()*(xList[k] - xgoal) +
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0);
                // cout << "CMAT SCALAR 1: " << c_mat_to_scalar << endl;
                // if(debugging_print) TRACE_KUKA_ARM("after the update3\n");
            }
            else {
                // if(debugging_print) TRACE_KUKA_ARM("before the update4\n");
                FList[k] = update(nargout_update2, xList[k], uList[k], AA, BB);//assume three outputs, code needs to be optimized
                // if(debugging_print) TRACE_KUKA_ARM("before the update4-1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal) +
                0.5*(xList[k].transpose() - xList_bar[k].transpose()) * costFunction->getRho_state() * (xList[k] - xList_bar[k]);;
                // if(debugging_print) TRACE_KUKA_ARM("after the update4\n");

                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k] +
                0.5*(uList[k].transpose() - uList_bar[k].transpose()) * costFunction->getRho_torque()* (uList[k] - uList_bar[k]);
                costFunction->getc() += c_mat_to_scalar(0,0); // TODO: to be checked
                // if(debugging_print) TRACE_KUKA_ARM("after the update5\n");
                // cout << "CMAT SCALAR 2: " << c_mat_to_scalar << endl;
                A_temp[k] = AA;
                B_temp[k] = BB;
            }
        }

        stateVec_t cx_temp;
        // cout << cx_temp << endl;

        if(debugging_print) TRACE_KUKA_ARM("compute dynamics and cost derivative\n");

        for(unsigned int k=0;k<Nl-1;k++){
            fxList[k] = A_temp[k];
            fuList[k] = B_temp[k];

            // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);

            cx_temp << xList[k] - xgoal;

            // if (k==0)
            //     cout << cx_temp << endl;

            costFunction->getcx()[k] = costFunction->getQ()*cx_temp + costFunction->getRho_state()*(xList[k]-xList_bar[k]);

            costFunction->getcu()[k] = costFunction->getR()*uList[k] + costFunction->getRho_torque()*(uList[k]-uList_bar[k]);
            costFunction->getcxx()[k] = costFunction->getQ() + costFunction->getRho_state();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR() + costFunction->getRho_torque();
        }
        if(debugging_print) TRACE_KUKA_ARM("update the final value of cost derivative \n");

        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal) + costFunction->getRho_state()*(xList[Nl-1]-xList_bar[Nl-1]);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1] + costFunction->getRho_torque()*(uList[Nl-1]-uList_bar[Nl-1]);
        costFunction->getcxx()[Nl-1] = costFunction->getQf() + costFunction->getRho_state();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR() + costFunction->getRho_torque();


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

void KukaArm_TRK::kuka_arm_dyn_cst_min_output(const int& nargout, const stateVec_t& xList_curr, const commandVec_t& uList_curr,
        const stateVec_t& xList_cur_bar, const commandVec_t& uList_cur_bar, const bool& isUNan, stateVec_t& xList_next, CostFunctionKukaArm_TRK*& costFunction){
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
            c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose()) * costFunction->getQf() * (xList_curr - xgoal) +
            0.5*(xList_curr.transpose() - xList_cur_bar.transpose())*costFunction->getRho_state()*(xList_curr - xList_cur_bar);
            costFunction->getc() += c_mat_to_scalar(0,0);
            // if(debugging_print) TRACE_KUKA_ARM("after the update1\n");
        }
        else {
            // if(debugging_print) TRACE_KUKA_ARM("before the update2\n");
            xList_next = update(nargout_update1, xList_curr, uList_curr, AA, BB);
            c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose())*costFunction->getQ()*(xList_curr - xgoal) +
            0.5*(xList_curr.transpose() - xList_cur_bar.transpose())*costFunction->getRho_state()*(xList_curr - xList_cur_bar);
            // if(debugging_print) TRACE_KUKA_ARM("after the update2\n");
            c_mat_to_scalar += 0.5*uList_curr.transpose()*costFunction->getR()*uList_curr +
            0.5*(uList_curr.transpose() - uList_cur_bar.transpose()) * costFunction->getRho_torque()* (uList_curr - uList_cur_bar);
            costFunction->getc() += c_mat_to_scalar(0,0);
        }
    }
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}

stateVec_t KukaArm_TRK::update(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    // 4th-order Runge-Kutta step
    if(debugging_print) TRACE_KUKA_ARM("update: 4th-order Runge-Kutta step\n");

    gettimeofday(&tbegin_period4,NULL);

    // output of kuka arm dynamics is xdot = f(x,u)
    Xdot1 = kuka_arm_dynamics(X, U);
    Xdot2 = kuka_arm_dynamics(X + 0.5*dt*Xdot1, U);
    Xdot3 = kuka_arm_dynamics(X + 0.5*dt*Xdot2, U);
    Xdot4 = kuka_arm_dynamics(X + dt*Xdot3, U);
    stateVec_t X_new;
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
        stateMat_t Dx;
        commandMat_t Du;
        Dx.setIdentity();
        Dx = delta*Dx;
        Du.setIdentity();
        Du = delta*Du;

        // State perturbation?
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

        // Control perturbation?
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
        B = B4 * dt/6 + (IdentityMat + A4 * dt/6) * B3 * dt/3 + (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)* B2 * dt/3 + (IdentityMat + (dt/6)*A4)*(IdentityMat + (dt/3)*A3)*(IdentityMat + (dt/3)*A2)*(dt/6)*B1;
    }
    if(debugging_print) TRACE_KUKA_ARM("update: X_new\n");

    gettimeofday(&tend_period4,NULL);
    finalTimeProfile.time_period4 += (static_cast<double>(1000.0*(tend_period4.tv_sec-tbegin_period4.tv_sec)+((tend_period4.tv_usec-tbegin_period4.tv_usec)/1000.0)))/1000.0;

    return X_new;
}

void KukaArm_TRK::grad(const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-7;
    stateMat_t Dx;
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
void KukaArm_TRK::hessian(const stateVec_t& X, const commandVec_t& U, stateTens_t& fxx_p, stateR_stateC_commandD_t& fxu_p, stateR_commandC_commandD_t& fuu_p){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-5;
    stateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    stateMat_t Ap;
    Ap.setZero();
    stateMat_t Am;
    Am.setZero();
    stateR_commandC_t B;
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

    stateR_commandC_t Bp;
    Bp.setZero();
    stateR_commandC_t Bm;
    Bm.setZero();

    for(unsigned int j=0;j<m;j++){
        grad(X, U+Du.col(j), Ap, Bp);
        grad(X, U-Du.col(j), Am, Bm);
        fxu_p[j] = (Ap - Am)/(2*delta);
        fuu_p[j] = (Bp - Bm)/(2*delta);
    }
}

unsigned int KukaArm_TRK::getStateNb()
{
    return stateNb;
}

unsigned int KukaArm_TRK::getCommandNb()
{
    return commandNb;
}

commandVec_t& KukaArm_TRK::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& KukaArm_TRK::getUpperCommandBounds()
{
    return upperCommandBounds;
}

stateMatTab_t& KukaArm_TRK::getfxList()
{
    return fxList;
}

stateR_commandC_tab_t& KukaArm_TRK::getfuList()
{
    return fuList;
}


void KukaArm_TRK::kuka_arm_dyn_cst_udp(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList,
                                CostFunctionKukaArm_TRK*& costFunction){
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
        stateVec_t cx_temp;
        if(debugging_print) TRACE_KUKA_ARM("compute cost derivative\n");
        for(unsigned int k=0;k<Nl-1;k++){
            cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
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
