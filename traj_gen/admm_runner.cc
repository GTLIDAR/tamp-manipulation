#include "drake/traj_gen/admm_runner.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {
void ADMMRunner::Initialize(unsigned int NKnot, unsigned int ADMMiterMax){
  // Initalize Primal and Dual variables
  res_x.resize(ADMMiterMax);
  res_x_ca.resize(ADMMiterMax);
  res_x_pos.resize(ADMMiterMax);
  res_x_vel.resize(ADMMiterMax);
  res_u.resize(ADMMiterMax);
  res_xlambda.resize(ADMMiterMax);
  res_xlambda_ca.resize(ADMMiterMax);
  res_ulambda.resize(ADMMiterMax);
  final_cost.resize(ADMMiterMax+1);
  xbar.resize(NKnot + 1);
  ubar.resize(NKnot);
  xbar_old.resize(NKnot + 1);
  ubar_old.resize(NKnot);
  xubar.resize(NKnot + 1);
  x_lambda.resize(NKnot + 1);
  x_lambda_ca.resize(NKnot + 1);
  u_lambda.resize(NKnot);
  x_temp.resize(NKnot+1);
  x_temp_ca.resize(NKnot+1);
  u_temp.resize(NKnot);
  x_temp2.resize(NKnot+1);
  u_temp2.resize(NKnot);

  for(unsigned int k=0;k<NKnot;k++){
  xbar[k].setZero();
  ubar[k].setZero();
  x_temp[k].setZero();
  x_temp_ca[k].setZero();
  u_temp[k].setZero();
  x_temp2[k].setZero();
  u_temp2[k].setZero();
  }
  xbar[NKnot].setZero();
  x_temp[NKnot].setZero();
  x_temp2[NKnot].setZero();
  x_temp_ca[NKnot].setZero();
}


lcmt_manipulator_traj ADMMRunner::RunADMM(stateVec_t xinit, stateVec_t xgoal,
  double time_horizon, double time_step, string action_name) {
    struct timeval tbegin,tend;
    double texec = 0.0;
    time_step_ = time_step;
    double dt = time_step;
    N = int(time_horizon/time_step);
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria

    unsigned int iterMax = 15;
    unsigned int ADMMiterMax = 100; 
    this->Initialize(N, ADMMiterMax);
    // if (action_name.compare("push")==0 || action_name.compare("throw")==0) {
    //   iterMax = 50;
    //   ADMMiterMax = 5;
    // }

    //======================================================================
    // Build wholebody and pass over to kukaArm
    std::string kIiwaUrdf =
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string schunkPath =
      FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf");
    std::string connectorPath =
      FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");

    std::string urdf_;
    auto plant_ = multibody::MultibodyPlant<double>(0.001);
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
    RigidTransformd X_EG(RollPitchYaw<double>(0, 0, M_PI_2),
                                Vector3d(0, 0, 0.0175));
    plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

    plant_.Finalize();
    
    #if WHOLE_BODY
      KukaArm_TRK KukaArmModel(dt, N, xgoal, &plant_, action_name);
    #else
      KukaArm_TRK KukaArmModel(dt, N, xgoal, action_name);
    #endif
    commandVecTab_t u_0;
    u_0.resize(N);
    for(unsigned i=0;i<N;i++){
      u_0[i] = KukaArmModel.quasiStatic(xinit);
    }


    // Initialize ILQRSolver
    ILQRSolver_TRK::traj lastTraj;
    pos_weight_ = 1e4;
    vel_weight_ = 1e3;
    torque_weight_ = 0;
    CostFunctionKukaArm_TRK costKukaArm_init(0, 0, 0, N); //only for initialization
    CostFunctionKukaArm_TRK costKukaArm_admm(pos_weight_, vel_weight_, torque_weight_, N); //postion/velocity/torque weights
    ILQRSolver_TRK testSolverKukaArm(KukaArmModel,costKukaArm_admm,ENABLE_FULLDDP,ENABLE_QPBOX);
    ILQRSolver_TRK testSolverKukaArm_init(KukaArmModel,costKukaArm_init,ENABLE_FULLDDP,ENABLE_QPBOX); //only for initialization

    // Initialize Trajectory to get xnew with u_0
    testSolverKukaArm_init.firstInitSolver(xinit, xgoal, xbar, ubar, u_0, N, dt, iterMax, tolFun, tolGrad);
    testSolverKukaArm_init.initializeTraj();

    lastTraj = testSolverKukaArm_init.getLastSolvedTrajectory();
    xnew = lastTraj.xList;
    unew = lastTraj.uList;
    final_cost[0] = lastTraj.finalCost;

    // Initialize some primal and dual parameters
    xnew_ca = xnew;

    for(unsigned int k=0;k<N;k++){
      x_lambda[k] = xnew[k] - xbar[k];
      x_lambda_ca[k] = xnew_ca[k] - xbar[k];
      u_lambda[k] = unew[k] - ubar[k];
    }
    x_lambda[N] = xnew[N] - xbar[N];
    x_lambda_ca[N] = xnew_ca[N] - xbar[N];

    std::vector<double> error_ca;
    error_ca.resize(ADMMiterMax);

    // Run ADMM
    cout << "\n=========== begin ADMM ===========\n";
    gettimeofday(&tbegin,NULL);
    for(unsigned int i=0;i<ADMMiterMax;i++){// TODO: Stopping criterion is needed
      for(unsigned int k=0;k<N;k++){
        x_temp[k] = xbar[k] - x_lambda[k];
        x_temp_ca[k] = xbar[k] - x_lambda_ca[k];
        u_temp[k] = ubar[k] - u_lambda[k];
      }
      x_temp[N] = xbar[N] - x_lambda[N];
      x_temp_ca[N] = xbar[N] - x_lambda_ca[N];

      cout << "\n=========== ADMM iteration " << i+1 << " ===========\n";
      /// iLQR solver block
      testSolverKukaArm.firstInitSolver(xinit, xgoal, x_temp, u_temp, unew, N, dt, iterMax, tolFun, tolGrad);
      testSolverKukaArm.solveTrajectory();
      lastTraj = testSolverKukaArm.getLastSolvedTrajectory();
      xnew = lastTraj.xList;
      unew = lastTraj.uList;

      cout << "\n=========== begin NLP for collision avoidance ===========\n";
      /// Collision avoidance block
      xnew_ca = CollisionAvoidance(plant_, x_temp_ca);
        // manually set the velocity back to xnew's just to maintain the same dimension for simplicity
      for(unsigned int k=0;k<=N;k++){
        xnew_ca[k].bottomRows(kNumJoints) = xnew[k].bottomRows(kNumJoints);
      }

      /// Projection block to feasible sets (state and control contraints)
      xbar_old = xbar;
      ubar_old = ubar;
      for(unsigned int k=0;k<N;k++){
        // Average joint position and corresponding dual variables from iLQR and CA blocks
        x_temp2[k] = (xnew[k] + x_lambda[k] + xnew_ca[k] + x_lambda_ca[k])/2;
        u_temp2[k] = unew[k] + u_lambda[k];
      }
      x_temp2[N] = (xnew[N] + x_lambda[N] + xnew_ca[N] + x_lambda_ca[N])/2;

      xubar = projection(x_temp2, u_temp2, N, action_name);

      // Dual variables update
      for(unsigned int j=0;j<N;j++){
        xbar[j] = xubar[j].head(stateSize);
        ubar[j] = xubar[j].tail(commandSize);
        // cout << "u_bar[" << j << "]:" << ubar[j].transpose() << endl;
        x_lambda[j] += xnew[j] - xbar[j];
        x_lambda_ca[j] += xnew_ca[j] - xbar[j];
        u_lambda[j] += unew[j] - ubar[j];

        // cout << "u_lambda[" << j << "]:" << u_lambda[j].transpose() << endl;

        // Save residuals for all iterations
        res_x[i] += (xnew[j] - xbar[j]).norm();
        res_x_ca[i] += (xnew_ca[j] - xbar[j]).norm();
        error_ca[i] += (xnew_ca[j] - xnew[j]).norm();
        res_x_pos[i] += (xnew[j].head(kNumJoints) - xbar[j].head(kNumJoints)).norm();
        res_x_vel[i] += (xnew[j].tail(7) - xbar[j].tail(7)).norm();
        // res_x_vel[i] += pow(xnew[j](7) - xbar[j](7), 2.0);
        res_u[i] += (unew[j] - ubar[j]).norm();

        res_xlambda[i] += vel_weight_*(xbar[j] - xbar_old[j]).norm();
        res_ulambda[i] += 0*(ubar[j] - ubar_old[j]).norm();
      }
      xbar[N] = xubar[N].head(stateSize);
      x_lambda[N] += xnew[N] - xbar[N];
      x_lambda_ca[N] += xnew_ca[N] - xbar[N];

      res_x[i] += (xnew[N] - xbar[N]).norm();
      res_x_ca[i] += (xnew_ca[N] - xbar[N]).norm();
      error_ca[i] += (xnew_ca[N] - xnew[N]).norm();
      res_x_pos[i] += (xnew[N].head(kNumJoints) - xbar[N].head(kNumJoints)).norm();
      res_x_vel[i] += (xnew[N].tail(7) - xbar[N].tail(7)).norm();
      // res_x_vel[i] += pow(xnew[N](7) - xbar[N](7), 2.0);
      res_xlambda[i] += 0*(xbar[N] - xbar_old[N]).norm();

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

    joint_state_traj.resize(N+1);
    joint_state_traj_ca.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    joint_state_traj_interp_ca.resize(N*InterpolationScale+1);
    position_traj_interp.resize(N*InterpolationScale+1);
    
    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = xnew[i];
      joint_state_traj_ca[i] = xnew_ca[i];
    }
    torque_traj = unew;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<stateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
       joint_state_traj_interp_ca[j](i,0) =  joint_state_traj_ca[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj_ca[index+1](i,0) - joint_state_traj_ca[index](i,0))/10.0;
       if(i<stateSize/2){
         position_traj_interp[j](i,0) = joint_state_traj_interp[j](i,0);
       }
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
      joint_state_traj_interp_ca[N*InterpolationScale](i,0) = joint_state_traj_ca[N](i,0);
      if(i<stateSize/2){
         position_traj_interp[N*InterpolationScale](i,0) = joint_state_traj_interp[N*InterpolationScale](i,0);
       }
    }

    texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    // texec /= Num_run;

    cout << endl;
    // cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    cout << "Final gradient: " << lastTraj.finalGrad << endl;
    cout << "Final lambda: " << lastTraj.finalLambda << endl;
    cout << "Execution time by time step (second): " << texec/N << endl;
    cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;



    cout << "lastTraj.xList[" << N << "]:" << xnew[N].transpose() << endl;
    cout << "lastTraj.uList[" << N-1 << "]:" << unew[N-1].transpose() << endl;

    cout << "lastTraj.xList[0]:" << xnew[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << unew[0].transpose() << endl;

    // Do the forward kinamtics for the EE to check the performance
    auto context_ptr = plant_.CreateDefaultContext();
    auto context = context_ptr.get();
    Vector2d wsg_width;
    wsg_width << -25, 25;
    for(unsigned int i=0;i<=N;i++){      
      auto rpy = math::RollPitchYawd(Eigen::Vector3d(0, 0, 0));
      auto xyz = Eigen::Vector3d(0, 0, 0);
      math::RigidTransform<double> X_W1(math::RotationMatrix<double>(rpy), xyz);
      plant_.SetPositions(context, iiwa_model, lastTraj.xList[i].topRows(7));
      plant_.SetPositions(context, wsg_model, wsg_width);
      plant_.SetVelocities(context, iiwa_model, lastTraj.xList[i].bottomRows(7));
      const auto& X_WB_all = plant_.get_body_poses_output_port().Eval<std::vector<math::RigidTransform<double>>>(*context);
      const BodyIndex ee_body_index = plant_.GetBodyByName("iiwa_link_ee_kuka", iiwa_model).index();
      const BodyIndex wsg_left_body_index = plant_.GetBodyByName("left_ball_contact3", wsg_model).index();
      const BodyIndex wsg_right_body_index = plant_.GetBodyByName("right_ball_contact3", wsg_model).index();
      const math::RigidTransform<double>& X_Wee = X_WB_all[ee_body_index];
      const math::RigidTransform<double>& X_Wwsg_l = X_WB_all[wsg_left_body_index];
      const math::RigidTransform<double>& X_Wwsg_r = X_WB_all[wsg_right_body_index];
      cout << "ee[" << i << "]:" << X_Wee.translation().transpose() << endl;
      cout << "wsg_l[" << i << "]:" << X_Wwsg_l.translation().transpose() << endl;
      cout << "wsg_r[" << i << "]:" << X_Wwsg_r.translation().transpose() << endl;
    }

    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory_ADMM_demo");
      saveVector(torque_traj[i], "joint_torque_command_ADMM_demo");
      saveVector(xubar[i], "xubar_ADMM_demo");
    }
    saveVector(joint_state_traj[N], "joint_trajectory_ADMM_demo");
    saveVector(xubar[N], "xubar_ADMM_demo");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated_ADMM_demo");
      saveVector(joint_state_traj_interp_ca[i], "xnew_ca_interpolated_ADMM_demo");
    }

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      saveValue(res_x_pos[i], "residual_x_pos_move_demo_nocontact");
      saveValue(res_x_vel[i], "residual_x_vel_move_demo_nocontact");
      saveValue(res_u[i], "residual_u_move_demo_nocontact");
    }
    cout << "-------- ADMM Trajectory Generation Finished! --------" << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x[" << i << "]:" << res_x[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x_ca[" << i << "]:" << res_x_ca[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "error_ca[" << i << "]:" << error_ca[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x_pos[" << i << "]:" << res_x_pos[i] << endl;
    }
    cout << endl;

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x_vel[" << i << "]:" << res_x_vel[i] << endl;
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

      for (int32_t j=0; j < ptr->dim_states; ++j) {
        ptr2->push_back(0);
        ptr2_st->push_back(joint_state_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
    }

    return *ptr;
  }

stateVecTab_t ADMMRunner::CollisionAvoidance(const drake::multibody::MultibodyPlant<double>& plant,
                                             const stateVecTab_t& X){
    /// Input consists of position + velocity
    // Define the optimization problem
    solvers::MathematicalProgram prog = solvers::MathematicalProgram();

    // Instantiate the decision variables
    auto x = prog.NewContinuousVariables(7, X.size(), "x");
    auto x0 = MatrixXd(7, X.size());
    x0.setZero();
    Vector1d lb, ub;
    lb << 0.12;
    ub << 100;
    Vector3d target;
    // target << 0.3856, 0.15, 0.40;
    target << 0.6, 0.05, 0.1;
    for(unsigned int i=0;i<X.size();i++){
        auto x_var = x.col(i);
        auto cost2 = prog.AddL2NormCost(MatrixXd::Identity(7,7), X[i].topRows(7), x_var);
        // Constraints that ensures the distance away from the obstacle
        prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint<double>>(plant, target, "iiwa", "iiwa_link_ee_kuka", 
                                        lb, std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK"), x_var);
        // prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint<double>>(plant, target, "wsg", "right_ball_contact3", 
        //                                 lb, std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK"), x_var);
        // prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint<double>>(plant, target, "wsg", "left_ball_contact3", 
        //                                 lb, std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK"), x_var);

        // Constraints to make the arm above the table
        prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint_z<double>>(plant, "iiwa", "iiwa_link_ee_kuka", 
                                        drake::Vector1d(0.1), std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK_z"), x_var);
        // prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint_z<double>>(plant, "wsg", "right_ball_contact3", 
        //                                 lb, std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK"), x_var);
        // prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint_z<double>>(plant, "wsg", "left_ball_contact3", 
        //                                 lb, std::numeric_limits<double>::infinity() * VectorXd::Ones(1), "FK"), x_var);
    }

    prog.SetInitialGuess(x, x0);
    auto result = solvers::Solve(prog);
    cout << "Is optimization successful? " << result.is_success() << endl;
    // cout << "Optimal x: " << result.GetSolution() << endl;
    // cout << "solver is: " << result.get_solver_id().name() << endl;
    
    // Rearrange the result into admm data type
    stateVecTab_t Y = X;
    // Y.resize(X.size());
    for(unsigned int i=0;i<Y.size();i++){
        Y[i].topRows(7) = result.GetSolution().middleRows<7>(7*i);
        // Y[i].bottomRows(7) = X[i].bottomRows(7);

    }
    return Y;
}

projStateAndCommandTab_t ADMMRunner::projection(const stateVecTab_t& X,
  const commandVecTab_t& U, unsigned int NumberofKnotPt,
  string action_name){
    projStateAndCommandTab_t XU_new;
    XU_new.resize(NumberofKnotPt+1);

    double joint_limit;
    // double vel_limit;
    // double vel_limit [] = {1.0, 1.0, 1.4, 2.0, 2.2, 2.5, 2.5};
    double vel_limit [] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; 
    // double torque_limit;
    double torque_limit [] = {150, 150, 80, 80, 80, 30, 30};
    // double torque_limit [] = {15, 15, 15, 15, 15, 15, 15};

    if (action_name.compare("throw")==0 || action_name.compare("push")==0) {
      joint_limit = 2.8;
      // vel_limit = 1.5;
      // torque_limit = 15;
    } else {
      joint_limit = 2.8;
      // vel_limit = 1.5;
      // torque_limit = 15;
    }

    for(unsigned int i=0;i<NumberofKnotPt+1;i++){
    for(unsigned int j=0;j<stateSize+commandSize;j++){
        if(j < stateSize/2){//postion constraints
        if(pos_weight_ > 0){
          if(X[i](j,0) > joint_limit){
              XU_new[i](j,0) = joint_limit;
          }
          else if(X[i](j,0) < -joint_limit){
              XU_new[i](j,0) = -joint_limit;
          }
          else{
              XU_new[i](j,0) = X[i](j,0);
          }
        }
        else{
            XU_new[i](j,0) = X[i](j,0);
        }
        }

        else if(j >= stateSize/2 && j < stateSize){//velocity constraints
        if(vel_weight_ > 0){
          if(X[i](j,0) > vel_limit[j-stateSize/2]){
              XU_new[i](j,0) = vel_limit[j-stateSize/2];
          }
          else if(X[i](j,0) < -vel_limit[j-stateSize/2]){
              XU_new[i](j,0) = -vel_limit[j-stateSize/2];
          }
          else{
              XU_new[i](j,0) = X[i](j,0);
          }
        }else{
            XU_new[i](j,0) = X[i](j,0);
        }
        }

        else{//torque constraints
        if(i<NumberofKnotPt){
          if(torque_weight_ > 0){
            if(U[i](j-stateSize,0) > torque_limit[j-stateSize]){
            XU_new[i](j,0) = torque_limit[j-14];
            }
            else if(U[i](j-stateSize,0) < -torque_limit[j-stateSize]){
            XU_new[i](j,0) = -torque_limit[j-stateSize];
            }
            else{
            XU_new[i](j,0) = U[i](j-stateSize,0);
            }
          }else{
            XU_new[i](j,0) = U[i](j-stateSize,0);
          }
        }
        else{
            XU_new[i].bottomRows(commandSize) = VectorXd::Zero(commandSize);
        }
        }
    }
    }
    // XU_new.assign(NumberofKnotPt+1, Eigen::VectorXd(stateSize+commandSize));
    return XU_new;
}

void ADMMRunner::RunVisualizer(double realtime_rate){
    lcm_.subscribe(kLcmTimeChannel,
                        &ADMMRunner::HandleRobotTime, this);
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

    VectorXd obstacle_pose(7);
    obstacle_pose <<  1.0, 0.0, 0.0, 0.0, 0.6, 0.05, 0.1;
    for (int joint = 0; joint < 7; joint++) 
    {
      object_state.joint_position_measured[joint] = obstacle_pose[joint];
    }
    
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
        // cout << step_ << endl;

        if(step_ >= N*InterpolationScale)
        {
            drake::log()->info("Interpolated trajectory has been published");
            plan_finished_ = true;
            break;
        }

        // pass the interpolated traj to lcm
        for (int32_t j=0; j < iiwa_state.num_joints; ++j) { 
            iiwa_state.joint_position_measured[j] = joint_state_traj_interp[step_][j];
        }

        lcm_.publish(kLcmStatusChannel, &iiwa_state);
        lcm_.publish(kLcmObjectStatusChannel, &object_state);
        lcm_.publish(kLcmSchunkStatusChannel, &wsg_status);
    }
}

void ADMMRunner::HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
        robot_time_ = *robot_time;
}

void ADMMRunner::saveVector(const Eigen::MatrixXd & _vec, const char * _name){
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

void ADMMRunner::saveValue(double _value, const char * _name){
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    save_file<<_value <<"\n";
    save_file.flush();
}

void ADMMRunner::clean_file(const char * _file_name, std::string & _ret_file){
    std::list<std::string>::iterator iter = std::find(admm_gs_filename_string.begin(), admm_gs_filename_string.end(), _file_name);
    if(admm_gs_filename_string.end() == iter){
        admm_gs_filename_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

} // kuka_iiwa_arm
} // traj_gen
} // namespace drake
