#include "drake/traj_gen/admm_contact_constraints/ddp_runner_contact_new.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

lcmt_manipulator_traj DDP_KKTRunner_new::RunDDP_KKT(fullstateVec_t xinit, fullstateVec_t xgoal, 
  double time_horizon, double time_step, string action_name) {
    struct timeval tbegin,tend;
    double texec = 0.0;

    time_step_ = time_step;
    double dt = time_step;
    N = int(time_horizon/time_step);
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 15; //100;

    ILQR_KKTSolver_new::traj lastTraj;
    //=============================================
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

    const ModelInstanceIndex object_model =
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

    // cout << "bias total" << endl << gtau_wb << endl;
    commandVecTab_t u_0;
    u_0.resize(N);
    for(unsigned i=0;i<N;i++){
    //   u_0[i] = -gtau_wb.middleRows<kNumJoints>(6);
        // cout << "u_0: " << u_0[i].transpose() << endl;
        u_0[i].setZero();
        // u_0[i] << 0, 0, 0, 0, 2, 2, 2;
    }
    //======================================
    KukaArm_Contact_new KukaArmModel(dt, N, xgoal, &plant_, action_name);
    CostFunctionKukaArm_Contact_new costKukaArm(N, action_name);
    ILQR_KKTSolver_new testSolverKukaArm(KukaArmModel,costKukaArm,ENABLE_FULLDDP,ENABLE_QPBOX);
    testSolverKukaArm.firstInitSolver(xinit, xgoal, u_0, N, dt, iterMax, tolFun, tolGrad);     

    // run one or multiple times and then average
    unsigned int Num_run = 1;
    gettimeofday(&tbegin,NULL);
    for(unsigned int i=0;i<Num_run;i++) {testSolverKukaArm.solveTrajectory();}
    if(Num_run == 0) {testSolverKukaArm.initializeTraj();}
    gettimeofday(&tend,NULL);

    lastTraj = testSolverKukaArm.getLastSolvedTrajectory();
    #if useUDPSolver
    finalTimeProfile = KukaArmModel.getFinalTimeProfile();
    #endif
    joint_state_traj.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    position_traj_interp.resize(N*InterpolationScale+1);

    for(unsigned int i=0;i<=N;i++){
    joint_state_traj[i] = lastTraj.xList[i];
    }
    torque_traj = lastTraj.uList;

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
    texec /= Num_run;

    cout << endl;
    cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    cout << "Final gradient: " << lastTraj.finalGrad << endl;
    cout << "Final lambda: " << lastTraj.finalLambda << endl;
    cout << "Execution time by time step (second): " << texec/N << endl;
    cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;

    cout << "xgoal: " << xgoal.transpose() << endl;
    cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList[N].transpose() << endl;
    cout << "lastTraj.uList[" << N-1 << "]:" << lastTraj.uList[N-1].transpose() << endl;

    cout << "lastTraj.xList[0]:" << lastTraj.xList[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << lastTraj.uList[0].transpose() << endl;

    
    // for(unsigned int i=N-50;i<=N;i++){
    //   cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    // }

    // Do the forward kinamtics for the EE to check the performance
    for(unsigned int i=N-2;i<=N;i++){      
        auto rpy = math::RollPitchYawd(Eigen::Vector3d(0, 0, 0));
        auto xyz = Eigen::Vector3d(0, 0, 0);
        math::RigidTransform<double> X_WO(math::RotationMatrix<double>(rpy), xyz);
        plant_.SetFreeBodyPoseInWorldFrame(context, plant_.GetBodyByName("base_link", object_model), X_WO);
        plant_.SetPositions(context, iiwa_model, lastTraj.xList[i].middleRows<7>(13));
        plant_.SetVelocities(context, iiwa_model, lastTraj.xList[i].bottomRows(7));
        const auto& X_WB_all = plant_.get_body_poses_output_port().Eval<std::vector<math::RigidTransform<double>>>(*context);
        const BodyIndex ee_body_index = plant_.GetBodyByName("iiwa_link_ee_kuka", iiwa_model).index();
        const math::RigidTransform<double>& X_Wee = X_WB_all[ee_body_index];
        cout << "ee[" << i << "]:" << X_Wee.translation().transpose() << endl;
    }

    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory_DDP");
      saveVector(torque_traj[i], "joint_torque_command_DDP");
      saveVector(lastTraj.forceList[i], "force_trajectory_DDP");
    }
    saveVector(lastTraj.xList[N], "joint_trajectory_DDP");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated_DDP");
    }

    cout << "-------- DDP Trajectory Generation Finished! --------" << endl;

    // need this for dynamic memory allocation (push_back)
    auto ptr = std::make_unique<lcmt_manipulator_traj>();
    
    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = N*InterpolationScale; 
    ptr->cost = lastTraj.finalCost;

    for (int32_t i=0; i < ptr->n_time_steps; ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(time_step*i/InterpolationScale));
      // cout << time_step*i/InterpolationScale << endl;

      // cout << ptr->times_sec[i] << endl;

      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=13; j < 13+ptr->dim_states; ++j) { 
        //  ptr2->push_back(lastTraj.uList[i][j]);
        // ptr2->push_back(gtau[j]);

        ptr2->push_back(0);
        ptr2_st->push_back(joint_state_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
    }

    return *ptr;
}

void DDP_KKTRunner_new::RunVisualizer(double realtime_rate){
    lcm_.subscribe(kLcmTimeChannel_DDP_new,
                        &DDP_KKTRunner_new::HandleRobotTime, this);
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

        lcm_.publish(kLcmStatusChannel_DDP_new, &iiwa_state);
        

        for (int joint = 0; joint < 7; joint++) 
        {
            object_state.joint_position_measured[joint] = joint_state_traj_interp[step_][joint];
        }

        lcm_.publish(kLcmObjectStatusChannel_DDP_new, &object_state);
        lcm_.publish(kLcmSchunkStatusChannel_DDP_new, &wsg_status);
    }
}

void DDP_KKTRunner_new::HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
        robot_time_ = *robot_time;
}

void DDP_KKTRunner_new::saveVector(const Eigen::MatrixXd & _vec, const char * _name) {
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    DDP_KKTRunner_new::clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    for (int i(0); i < _vec.rows(); ++i){
        save_file<<_vec(i,0)<< "\t";
    }
    save_file<<"\n";
    save_file.flush();
    save_file.close();
}

void DDP_KKTRunner_new::saveValue(double _value, const char * _name){
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    DDP_KKTRunner_new::clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    save_file<<_value <<"\n";
    save_file.flush();
}

void DDP_KKTRunner_new::clean_file(const char * _file_name, std::string & _ret_file){
    std::list<std::string>::iterator iter = std::find(gs_kkt_new_fileName_string.begin(), gs_kkt_new_fileName_string.end(), _file_name);
    if(gs_kkt_new_fileName_string.end() == iter){
        gs_kkt_new_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

} //namespace kuka_iiwa_arm
} //namespace traj_gen
} //namespace drake