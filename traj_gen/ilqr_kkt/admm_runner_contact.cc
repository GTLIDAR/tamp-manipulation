#include "drake/traj_gen/ilqr_kkt/admm_runner_contact.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
lcmt_manipulator_traj ADMM_KKTRunner::RunADMM_KKT(fullstateVec_t xinit, fullstateVec_t xgoal,
  double time_horizon, double time_step, string action_name) {
    struct timeval tbegin,tend;
    double texec = 0.0;
    commandVecTab_t u_0;
    time_step_ = time_step;
    double dt = time_step;
    N = int(time_horizon/time_step);
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria

    unsigned int iterMax = 15;
    unsigned int ADMMiterMax = 5;

    if (time_horizon <= 1.5) {
      ADMMiterMax = 20;
    }

    // if (action_name.compare("push")==0 || action_name.compare("throw")==0) {
    //   iterMax = 50;
    //   ADMMiterMax = 5;
    // }


    // Initalize Primal and Dual variables
    // Primal
    fullstateVecTab_t xnew;
    commandVecTab_t unew;
    fullstateVecTab_t xbar;
    commandVecTab_t ubar;
    fullstateVecTab_t xbar_old;
    commandVecTab_t ubar_old;

    // Dual
    fullstateVecTab_t x_lambda;
    commandVecTab_t u_lambda;

    fullstateVecTab_t x_temp;
    commandVecTab_t u_temp;
    fullstateVecTab_t x_temp2;
    commandVecTab_t u_temp2;
    projfullStateAndCommandTab_t xubar;
    vector<double> res_x;
    vector<double> res_x_pos_obj;
    vector<double> res_x_pos_iiwa;
    vector<double> res_x_vel_obj;
    vector<double> res_x_vel_iiwa;
    vector<double> res_u;
    vector<double> res_xlambda;
    vector<double> res_ulambda;
    vector<double> final_cost;
    res_x.resize(ADMMiterMax);
    res_x_pos_obj.resize(ADMMiterMax);
    res_x_vel_obj.resize(ADMMiterMax);
    res_x_pos_iiwa.resize(ADMMiterMax);
    res_x_vel_iiwa.resize(ADMMiterMax);
    res_u.resize(ADMMiterMax);
    res_xlambda.resize(ADMMiterMax);
    res_ulambda.resize(ADMMiterMax);
    final_cost.resize(ADMMiterMax+1);
    xbar.resize(N + 1);
    ubar.resize(N);
    xbar_old.resize(N + 1);
    ubar_old.resize(N);
    xubar.resize(N + 1);
    u_0.resize(N);
    x_lambda.resize(N + 1);
    u_lambda.resize(N);
    x_temp.resize(N+1);
    u_temp.resize(N);
    x_temp2.resize(N+1);
    u_temp2.resize(N);

    for(unsigned int k=0;k<N;k++){
      xbar[k].setZero();
      ubar[k].setZero();
      x_temp[k].setZero();
      u_temp[k].setZero();
      x_temp2[k].setZero();
      u_temp2[k].setZero();
      u_0[k] << 0,0,0.2,0,0.2,0,0;
    }
    xbar[N].setZero();
    x_temp[N].setZero();
    x_temp2[N].setZero();

    //======================================================================
    // Build wholebody and pass over to kukaArm
    std::string kIiwaUrdf = 
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string schunkPath = 
        FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf");
    std::string connectorPath = 
        FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");

    std::string box_sdf_path0;
    if (action_name.compare("push")==0){
        box_sdf_path0 = FindResourceOrThrow("drake/manipulation_tamp/models/boxes/large_red_box.urdf");
    }
    else{
        box_sdf_path0 = FindResourceOrThrow("drake/manipulation_tamp/models/boxes/redblock.urdf");
    }

    std::string urdf_;
    auto plant_ = multibody::MultibodyPlant<double>(0.0);
    multibody::Parser parser(&plant_);
    
    const ModelInstanceIndex iiwa_model = 
        parser.AddModelFromFile(kIiwaUrdf, "iiwa");
    const auto& iiwa_base_frame = plant_.GetFrameByName("iiwa_link_0", iiwa_model);
    RigidTransformd X_WI(Eigen::Vector3d(0, 0, 0));
    plant_.WeldFrames(plant_.world_frame(), iiwa_base_frame, X_WI);

    const ModelInstanceIndex conn_model = 
        parser.AddModelFromFile(connectorPath, "connector");
    const auto& iiwa_ee_frame = plant_.GetFrameByName("iiwa_frame_ee", iiwa_model);
    const auto& conn_frame = plant_.GetFrameByName("connector_link", conn_model);
    RigidTransformd X_EC(Eigen::Vector3d(0, 0, 0));
    plant_.WeldFrames(iiwa_ee_frame, conn_frame, X_EC);

    const ModelInstanceIndex wsg_model = 
        parser.AddModelFromFile(schunkPath, "wsg");
    const auto& wsg_frame = plant_.GetFrameByName("body", wsg_model);
    RigidTransformd X_EG(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.053));
    plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

    // const ModelInstanceIndex object_model =
    parser.AddModelFromFile(box_sdf_path0, "object");

    plant_.Finalize();

    auto context_ptr = plant_.CreateDefaultContext();
    auto context = context_ptr.get();

    VectorXd q_v_iiwa(14);
    q_v_iiwa.setZero();
    q_v_iiwa.head(7) = xinit;
    plant_.SetPositionsAndVelocities(context, iiwa_model, q_v_iiwa);

    MatrixXd M_(plant_.num_velocities(), plant_.num_velocities());
    plant_.CalcMassMatrix(*context, &M_);

    VectorXd gtau_wb = plant_.CalcGravityGeneralizedForces(*context);

    u_0.resize(N);
    for(unsigned i=0;i<N;i++){
      u_0[i] = -gtau_wb.middleRows<kNumJoints>(6);
        // cout << "u_0: " << u_0[i].transpose() << endl;
        // u_0[i].setZero();
        // u_0[i] << 10, 10, 10, 10, 10, 10, 10;
    }
    
    //////////////////////////////////////////////////////////////////
    KukaArm_TRK_Contact KukaArmModel(dt, N, xgoal, &plant_, action_name);

    // Initialize ILQRSolver
    ILQRSolver_TRK_Contact::traj lastTraj;
    // KukaArm_TRK KukaArmModel(dt, N, xgoal);
    double pos_obj_weight; 
    double pos_iiwa_weight; 
    double vel_obj_weight;
    double vel_iiwa_weight;
    double torque_weight;

    pos_obj_weight = 0;
    pos_iiwa_weight = 10; 
    vel_obj_weight = 0;
    vel_iiwa_weight = 10;
    torque_weight = 1;

    CostFunctionKukaArm_TRK_Contact costKukaArm_init(0, 0, 0, 0, 0, N, action_name); //only for initialization
    CostFunctionKukaArm_TRK_Contact costKukaArm_admm(pos_obj_weight, pos_iiwa_weight, 
                                                     vel_obj_weight, vel_iiwa_weight, 
                                                     torque_weight, N, action_name); //postion/velocity/torque weights
    ILQRSolver_TRK_Contact testSolverKukaArm(KukaArmModel,costKukaArm_admm,ENABLE_FULLDDP,ENABLE_QPBOX);
    ILQRSolver_TRK_Contact testSolverKukaArm_init(KukaArmModel,costKukaArm_init,ENABLE_FULLDDP,ENABLE_QPBOX); //only for initialization

    // Initialize Trajectory to get xnew with u_0
    testSolverKukaArm_init.firstInitSolver(xinit, xgoal, xbar, ubar, u_0, N, dt, iterMax, tolFun, tolGrad);
    testSolverKukaArm_init.initializeTraj();

    lastTraj = testSolverKukaArm_init.getLastSolvedTrajectory();
    xnew = lastTraj.xList;
    unew = lastTraj.uList;
    final_cost[0] = lastTraj.finalCost;

    for(unsigned int k=0;k<N;k++){
      x_lambda[k] = xnew[k] - xbar[k];
      u_lambda[k] = unew[k] - ubar[k];
    }
    x_lambda[N] = xnew[N] - xbar[N];


    // Run ADMM
    cout << "\n=========== begin ADMM ===========\n";
    gettimeofday(&tbegin,NULL);
    for(unsigned int i=0;i<ADMMiterMax;i++){// TODO: Stopping criterion is needed
      // TODO: "-" operator for stateVecTab?
      for(unsigned int k=0;k<N;k++){
        x_temp[k] = xbar[k] - x_lambda[k];
        u_temp[k] = ubar[k] - u_lambda[k];
      }
      x_temp[N] = xbar[N] - x_lambda[N];

      // cout << "checkpoint 0" << endl;
      cout << "\n=========== ADMM iteration " << i+1 << " ===========\n";
      // iLQR solver block
      testSolverKukaArm.firstInitSolver(xinit, xgoal, x_temp, u_temp, unew, N, dt, iterMax, tolFun, tolGrad);
      testSolverKukaArm.solveTrajectory();
      lastTraj = testSolverKukaArm.getLastSolvedTrajectory();
      xnew = lastTraj.xList;
      unew = lastTraj.uList;

      //////////////////////////// Projection block to feasible sets (state and control contraints)
      xbar_old = xbar;
      ubar_old = ubar;
      for(unsigned int k=0;k<N;k++){
        x_temp2[k] = xnew[k] + x_lambda[k];
        u_temp2[k] = unew[k] + u_lambda[k];
      }
      x_temp2[N] = xnew[N] + x_lambda[N];
      xubar = projection(x_temp2, u_temp2, N, action_name);

      /////////////////////////// Dual variables update
      for(unsigned int j=0;j<N;j++){
        xbar[j] = xubar[j].head(fullstateSize);
        ubar[j] = xubar[j].tail(commandSize);
        // cout << "u_bar[" << j << "]:" << ubar[j].transpose() << endl;
        x_lambda[j] += xnew[j] - xbar[j];
        u_lambda[j] += unew[j] - ubar[j];
        // cout << "u_lambda[" << j << "]:" << u_lambda[j].transpose() << endl;

        // Save residuals for all iterations
        res_x[i] += (xnew[j] - xbar[j]).norm();
        res_x_pos_obj[i] += (xnew[j].head(7) - xbar[j].head(7)).norm();
        res_x_vel_obj[i] += (xnew[j].middleRows<6>(7) - xbar[j].middleRows<6>(7)).norm();
        res_x_pos_iiwa[i] += (xnew[j].middleRows<kNumJoints>(13) - xbar[j].middleRows<kNumJoints>(13)).norm();
        res_x_vel_iiwa[i] += (xnew[j].tail(kNumJoints) - xbar[j].tail(kNumJoints)).norm();
        res_u[i] += (unew[j] - ubar[j]).norm();

        res_xlambda[i] += 0*(xbar[j] - xbar_old[j]).norm(); //not used now
        res_ulambda[i] += 0*(ubar[j] - ubar_old[j]).norm(); // not used now
      }
      xbar[N] = xubar[N].head(fullstateSize);
      x_lambda[N] += xnew[N] - xbar[N];

      res_x[i] += (xnew[N] - xbar[N]).norm();
      res_x_pos_obj[i] += (xnew[N].head(7) - xbar[N].head(7)).norm();
      res_x_vel_obj[i] += (xnew[N].middleRows<6>(7) - xbar[N].middleRows<6>(7)).norm();
      res_x_pos_iiwa[i] += (xnew[N].middleRows<kNumJoints>(13) - xbar[N].middleRows<kNumJoints>(13)).norm();
      res_x_vel_iiwa[i] += (xnew[N].tail(kNumJoints) - xbar[N].tail(kNumJoints)).norm();
      res_xlambda[i] += 0*(xbar[N] - xbar_old[N]).norm(); // not used now

      // get the cost without augmented Lagrangian terms
      testSolverKukaArm_init.firstInitSolver(xinit, xgoal, xbar, ubar, unew, N, dt, iterMax, tolFun, tolGrad);
      testSolverKukaArm_init.initializeTraj();
      lastTraj = testSolverKukaArm_init.getLastSolvedTrajectory();
      final_cost[i+1] = lastTraj.finalCost;
      // cout << "checkpoint 3" << endl;
      if (isnan(lastTraj.finalCost)) {
        std::cout<<"NaN Final Cost. Stopping ADMM...\n";
        break;
      }
    }
    gettimeofday(&tend,NULL);

    testSolverKukaArm.firstInitSolver(xinit, xgoal, xbar, ubar, unew, N, dt, iterMax, tolFun, tolGrad);
    testSolverKukaArm.initializeTraj();
    xnew = testSolverKukaArm.updatedxList;

    #if useUDPSolver
      finalTimeProfile = KukaArmModel.getFinalTimeProfile();
    #endif
    joint_state_traj.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    position_traj_interp.resize(N*InterpolationScale+1);

    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = xnew[i];
    }
    torque_traj = unew;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<fullstateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
       if(i>12 && i<13+stateSize/2){
         position_traj_interp[j](i-13,0) = joint_state_traj_interp[j](i,0);
       }
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
      if(i>12 && i<13+stateSize/2){
         position_traj_interp[N*InterpolationScale](i-13,0) = joint_state_traj_interp[N*InterpolationScale](i,0);
       }
    }

    texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    // texec /= Num_run;

    cout << endl;
    // cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    // cout << "Final gradient: " << lastTraj.finalGrad << endl;
    // cout << "Final lambda: " << lastTraj.finalLambda << endl;
    // cout << "Execution time by time step (second): " << texec/N << endl;
    // cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    // cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    // cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;

    cout << "lastTraj.xList[" << N << "]:" << xnew[N].transpose() << endl;
    cout << "lastTraj.uList[" << N-1 << "]:" << unew[N-1].transpose() << endl;

    cout << "lastTraj.xList[0]:" << xnew[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << unew[0].transpose() << endl;

    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory_ADMM_contact_demo");
      saveVector(torque_traj[i], "joint_torque_command_ADMM_contact_demo");
      saveVector(xubar[i], "xubar_ADMM_contact_demo");
    }
    saveVector(xnew[N], "joint_trajectory_ADMM_contact_demo");
    saveVector(xubar[N], "xubar_ADMM_contact_demo");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated_ADMM_contact_demo");
    }

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      saveValue(res_x_pos_iiwa[i], "residual_x_pos_push_demo");
      saveValue(res_x_vel_iiwa[i], "residual_x_vel_push_demo");
      saveValue(res_u[i], "residual_u_push_demo");
    }
    cout << "-------- ADMM Trajectory Generation Finished! --------" << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x[" << i << "]:" << res_x[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x_pos_iiwa[" << i << "]:" << res_x_pos_iiwa[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x_vel_iiwa[" << i << "]:" << res_x_vel_iiwa[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_u[" << i << "]:" << res_u[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<=ADMMiterMax;i++)
    {
      cout << "final_cost[" << i << "]:" << final_cost[i] << endl;
    }

    // need this for dynamic memory allocation (push_back)
    auto ptr = std::make_unique<lcmt_manipulator_traj>();

    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = N*InterpolationScale;
    //ptr->cost = lastTraj.finalCost;
    ptr->cost = final_cost[ADMMiterMax];
      //============================================

    for (int32_t i=0; i < ptr->n_time_steps; ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(time_step*i/InterpolationScale));
      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=13; j < 13+ptr->dim_states; ++j) {
        ptr2->push_back(0);
        ptr2_st->push_back(joint_state_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
    }

    return *ptr;
  }

projfullStateAndCommandTab_t ADMM_KKTRunner::projection(const fullstateVecTab_t& xnew,
  const commandVecTab_t& unew, unsigned int NumberofKnotPt,
  string action_name){
    projfullStateAndCommandTab_t xubar;
    xubar.resize(NumberofKnotPt+1);

    double pos_limit_obj;
    double vel_limit_obj;
    double joint_limit_iiwa;
    // double vel_limit_iiwa;
    // double torque_limit;
    double vel_limit_iiwa [] = {1.0, 1.0, 1.4, 2.0, 2.2, 2.5, 2.5}; 
    double torque_limit [] = {100, 100, 50, 50, 50, 30, 30};

    if (action_name.compare("throw")==0 || action_name.compare("push")==0) {
      pos_limit_obj = 10; // not used for object now
      vel_limit_obj = 10;
      joint_limit_iiwa = 3.0;
      // vel_limit_iiwa = 1.5;
      // torque_limit = 30;
    } else {
      pos_limit_obj = 10; // not used for object now
      vel_limit_obj = 10;
      joint_limit_iiwa = 3.0;
      // vel_limit_iiwa = 1.5;
      // torque_limit = 30;
    }

    for(unsigned int i=0;i<NumberofKnotPt+1;i++){
    for(unsigned int j=0;j<fullstateSize+commandSize;j++){
        if(j < 7){//postion constraints for obj
        if(xnew[i](j,0) > pos_limit_obj){
            xubar[i](j,0) = pos_limit_obj;
        }
        else if(xnew[i](j,0) < -pos_limit_obj){
            xubar[i](j,0) = -pos_limit_obj;
        }
        else{
            xubar[i](j,0) = xnew[i](j,0);
        }
        }

        else if(j >= 7 && j < 13){//velocity constraints for obj

        if(xnew[i](j,0) > vel_limit_obj){
            xubar[i](j,0) = vel_limit_obj;
        }
        else if(xnew[i](j,0) < -vel_limit_obj){
            xubar[i](j,0) = -vel_limit_obj;
        }
        else{
            xubar[i](j,0) = xnew[i](j,0);
        }
        }

        else if(j >= 13 && j < 20){//position constraints for iiwa

        if(xnew[i](j,0) > joint_limit_iiwa){
            xubar[i](j,0) = joint_limit_iiwa;
        }
        else if(xnew[i](j,0) < -joint_limit_iiwa){
            xubar[i](j,0) = -joint_limit_iiwa;
        }
        else{
            xubar[i](j,0) = xnew[i](j,0);
        }
        }

        else if(j >= 20 && j < 27){//velocity constraints for iiwa

        if(xnew[i](j,0) > vel_limit_iiwa[j-20]){
            xubar[i](j,0) = vel_limit_iiwa[j-20];
        }
        else if(xnew[i](j,0) < -vel_limit_iiwa[j-20]){
            xubar[i](j,0) = -vel_limit_iiwa[j-20];
        }
        else{
            xubar[i](j,0) = xnew[i](j,0);
        }
        }

        else{//torque constraints
        if(i<NumberofKnotPt){
            if(unew[i](j-27,0) > torque_limit[j-27]){
            xubar[i](j,0) = torque_limit[j-27];
            }
            else if(unew[i](j-27,0) < -torque_limit[j-27]){
            xubar[i](j,0) = -torque_limit[j-27];
            }
            else{
            xubar[i](j,0) = unew[i](j-27,0);
            }
        }
        else{
            xubar[i].bottomRows(commandSize) = VectorXd::Zero(commandSize);
        }
        }
    }
    }
    return xubar;
}

void ADMM_KKTRunner::RunVisualizer(double realtime_rate){
    lcm_.subscribe(kLcmTimeChannel_ADMM,
                        &ADMM_KKTRunner::HandleRobotTime, this);
    lcmt_iiwa_status iiwa_state;
    lcmt_schunk_wsg_status wsg_status;
    lcmt_object_status object_state;
    iiwa_state.num_joints = kIiwaArmNumJoints;
    object_state.num_joints = 7;
    iiwa_state.joint_position_measured.resize(kIiwaArmNumJoints, 0.);   
    iiwa_state.joint_velocity_estimated.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_position_commanded.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_position_ipo.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_measured.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_commanded.resize(kIiwaArmNumJoints, 0.);
    iiwa_state.joint_torque_external.resize(kIiwaArmNumJoints, 0.);
    
    //////////////// !!!!!!!!!! /////////////////////
    // Warning: just for visualization since the gripper_position_output_port has been depricated in latest drake update
    // actual_position_mm = finger1 -- negative
    // actual_speed_mm_per_s = finger2 -- positive
    // maximum width = 110

    wsg_status.actual_position_mm = -25;
    wsg_status.actual_speed_mm_per_s = 25;
    
    object_state.joint_position_measured.resize(7, 0.);   
    object_state.joint_velocity_estimated.resize(7, 0.);
    object_state.joint_position_commanded.resize(7, 0.);
    object_state.joint_position_ipo.resize(7, 0.);
    object_state.joint_torque_measured.resize(7, 0.);
    object_state.joint_torque_commanded.resize(7, 0.);
    object_state.joint_torque_external.resize(7, 0.);

    drake::log()->info("Publishing trajectory to visualizer");
    plan_finished_ = false;

    while(true){
        while (0 == lcm_.handleTimeout(10) || iiwa_state.utime == -1 
        || plan_finished_) { }

        // if(!start_publish){
        //   start_time = robot_time_.utime;     
        //   cout << "start_time: " << start_time << endl; 
        //   start_publish = true;
        // }

        // Update status time to simulation time
        // Note: utime is in microseconds
        iiwa_state.utime = robot_time_.utime;
        wsg_status.utime = robot_time_.utime;
        // step_ = int((robot_time_.utime / 1000)*(kIiwaLcmStatusPeriod/(time_step/InterpolationScale)));
        step_ = int(((robot_time_.utime) / 1000)*(0.001*realtime_rate/(time_step_/InterpolationScale)));
        // std::cout << step_ << std::endl;
        
        if(step_ >= N*InterpolationScale)
        {
            drake::log()->info("Interpolated trajectory has been published");
            plan_finished_ = true;
            break;
        }

        // pass the interpolated traj to lcm
        for (int32_t j=0; j < iiwa_state.num_joints; ++j) { 
            iiwa_state.joint_position_measured[j] = joint_state_traj_interp[step_][13 + j];
        }

        lcm_.publish(kLcmStatusChannel_ADMM, &iiwa_state);
        

        for (int joint = 0; joint < 7; joint++) 
        {
            object_state.joint_position_measured[joint] = joint_state_traj_interp[step_][joint];
        }

        lcm_.publish(kLcmObjectStatusChannel_ADMM, &object_state);
        lcm_.publish(kLcmSchunkStatusChannel_ADMM, &wsg_status);
    }
}

void ADMM_KKTRunner::HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
        robot_time_ = *robot_time;
}

void ADMM_KKTRunner::saveVector(const Eigen::MatrixXd & _vec, const char * _name){
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    for (int i(0); i < _vec.rows(); ++i){
        save_file<<_vec(i,0)<< "\t";
    }
    save_file<<"\n";
    save_file.flush();
    save_file.close();
}

void ADMM_KKTRunner::saveValue(double _value, const char * _name){
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    save_file<<_value <<"\n";
    save_file.flush();
}

void ADMM_KKTRunner::clean_file(const char * _file_name, std::string & _ret_file){
    std::list<std::string>::iterator iter = std::find(admm_kkt_gs_filename_string.begin(), admm_kkt_gs_filename_string.end(), _file_name);
    if(admm_kkt_gs_filename_string.end() == iter){
        admm_kkt_gs_filename_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

} // kuka_iiwa_arm
} // traj_gen
} // namespace drake
