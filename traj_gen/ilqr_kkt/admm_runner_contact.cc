#include "drake/traj_gen/ilqr_kkt/admm_runner_contact.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
lcmt_manipulator_traj ADMM_KKTRunner::RunADMM(stateVec_t xinit, stateVec_t xgoal,
  double time_horizon, double time_step, string action_name) {
    struct timeval tbegin,tend;
    double texec = 0.0;
    commandVecTab_t u_0;
    double dt = time_step;
    unsigned int N = int(time_horizon/time_step);
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria

    unsigned int iterMax = 15;
    unsigned int ADMMiterMax = 20;

    // if (action_name.compare("push")==0 || action_name.compare("throw")==0) {
    //   iterMax = 50;
    //   ADMMiterMax = 5;
    // }


    // Initalize Primal and Dual variables
    // Primal
    stateVecTab_t xnew;
    commandVecTab_t unew;
    stateVecTab_t xbar;
    commandVecTab_t ubar;
    stateVecTab_t xbar_old;
    commandVecTab_t ubar_old;

    // Dual
    stateVecTab_t x_lambda;
    commandVecTab_t u_lambda;

    stateVecTab_t x_temp;
    commandVecTab_t u_temp;
    stateVecTab_t x_temp2;
    commandVecTab_t u_temp2;
    projStateAndCommandTab_t xubar;
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
        box_sdf_path0 = FindResourceOrThrow("drake/conveyor_belt_tamp/models/boxes/large_red_box.urdf");
    }
    else{
        box_sdf_path0 = FindResourceOrThrow("drake/conveyor_belt_tamp/models/boxes/redblock.urdf");
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
    //   u_0[i] = -gtau_wb.middleRows<kNumJoints>(6);
        // cout << "u_0: " << u_0[i].transpose() << endl;
        u_0[i].setZero();
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
    pos_iiwa_weight = 20; 
    vel_obj_weight = 0;
    vel_iiwa_weight = 0;
    torque_weight = 20;

    CostFunctionKukaArm_TRK_Contact costKukaArm_init(0, 0, 0, 0, 0, N); //only for initialization
    CostFunctionKukaArm_TRK_Contact costKukaArm_admm(pos_obj_weight, pos_iiwa_weight, 
                                                     vel_obj_weight, vel_iiwa_weight, 
                                                     torque_weight, N); //postion/velocity/torque weights
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
        xbar[j] = xubar[j].head(stateSize);
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
      xbar[N] = xubar[N].head(stateSize);
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
    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = xnew[i];
    }
    torque_traj = unew;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<stateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
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

    #if useUDPSolver
    cout << "\t\tUDP backward propagation (second): " << lastTraj.time_range1.sum() << " (" << 100.0*lastTraj.time_range1.sum()/texec << "%)" << endl;
    cout << "\t\t\t20 multi-threading computation (second): " << lastTraj.time_range2.sum() << " (" << 100.0*lastTraj.time_range2.sum()/texec << "%)" << endl;
    cout << "\t\t\tMain thread computation (second): " << lastTraj.time_range3.sum() << " (" << 100.0*lastTraj.time_range3.sum()/texec << "%)" << endl;
    cout << "\tTime of forward pass (second): " << lastTraj.time_forward.sum() << " (" << 100.0*lastTraj.time_forward.sum()/texec << "%)" << endl;
    cout << "\tTotal number of a forward dynamics in main thread: " << finalTimeProfile.counter0_ << endl;
    cout << "\tTotal number of a forward dynamics in one worker thread: " << finalTimeProfile.counter1_ << endl;
    cout << "\tTotal number of a forward dynamics in one worker thread: " << finalTimeProfile.counter2_ << endl;
    // cout << "-----------" << endl;
    cout << "\tTime of one RBT block in main thread (second): " << finalTimeProfile.time_period1 << " (" << 100.0*finalTimeProfile.time_period1/texec << "%)" << endl;
    cout << "\tTime of one RBT block in one worker thread (second): " << finalTimeProfile.time_period2 << " (" << 100.0*finalTimeProfile.time_period2/texec << "%)" << endl;
    cout << "\tTime of one RBT block in one worker thread (second): " << finalTimeProfile.time_period3 << " (" << 100.0*finalTimeProfile.time_period3/texec << "%)" << endl;
    cout << "\tTime of update func (second): " << finalTimeProfile.time_period4 << " (" << 100.0*finalTimeProfile.time_period4/texec << "%)" << endl;
    #endif


    cout << "lastTraj.xList[" << N << "]:" << xnew[N].transpose() << endl;
    cout << "lastTraj.uList[" << N-1 << "]:" << unew[N-1].transpose() << endl;

    cout << "lastTraj.xList[0]:" << xnew[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << unew[0].transpose() << endl;

    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory_ADMM");
      saveVector(torque_traj[i], "joint_torque_command_ADMM");
    }
    saveVector(xnew[N], "joint_trajectory_ADMM");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated_ADMM");
    }

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      saveValue(res_x_pos_iiwa[i], "residual_x_pos_push");
      saveValue(res_x_vel_iiwa[i], "residual_x_vel_push");
      saveValue(res_u[i], "residual_u_push");
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

projStateAndCommandTab_t ADMM_KKTRunner::projection(const stateVecTab_t& xnew,
  const commandVecTab_t& unew, unsigned int NumberofKnotPt,
  string action_name){
    projStateAndCommandTab_t xubar;
    xubar.resize(NumberofKnotPt+1);

    double pos_limit_obj;
    double vel_limit_obj;
    double joint_limit_iiwa;
    double vel_limit_iiwa;
    double torque_limit;

    if (action_name.compare("throw")==0 || action_name.compare("push")==0) {
      pos_limit_obj = 10; // not used for object now
      vel_limit_obj = 10;
      joint_limit_iiwa = 2.0;
      vel_limit_iiwa = 1.5;
      torque_limit = 30;
    } else {
      pos_limit_obj = 10; // not used for object now
      vel_limit_obj = 10;
      joint_limit_iiwa = 3.0;
      vel_limit_iiwa = 1.5;
      torque_limit = 30;
    }

    for(unsigned int i=0;i<NumberofKnotPt+1;i++){
    for(unsigned int j=0;j<stateSize+commandSize;j++){
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

        else if(j >= 7 && 13){//velocity constraints for obj

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

        if(xnew[i](j,0) > vel_limit_iiwa){
            xubar[i](j,0) = vel_limit_iiwa;
        }
        else if(xnew[i](j,0) < -vel_limit_iiwa){
            xubar[i](j,0) = -vel_limit_iiwa;
        }
        else{
            xubar[i](j,0) = xnew[i](j,0);
        }
        }

        else{//torque constraints
        if(i<NumberofKnotPt){
            if(unew[i](j,0) > torque_limit){
            xubar[i](j,0) = torque_limit;
            }
            else if(unew[i](j,0) < -torque_limit){
            xubar[i](j,0) = -torque_limit;
            }
            else{
            xubar[i](j,0) = unew[i](j,0);
            }
        }
        else{
            xubar[i].setZero();
        }
        }
    }
    }
    return xubar;
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
    std::list<std::string>::iterator iter = std::find(admm_gs_filename_string.begin(), admm_gs_filename_string.end(), _file_name);
    if(admm_gs_filename_string.end() == iter){
        admm_gs_filename_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

} // kuka_iiwa_arm
} // traj_gen
} // namespace drake
