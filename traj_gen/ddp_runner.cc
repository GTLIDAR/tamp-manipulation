#include "drake/traj_gen/ddp_runner.h"

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

lcmt_ddp_traj DDPRunner::RunUDP(stateVec_t xinit, stateVec_t xgoal, 
  const lcmt_motion_plan_query* query) {
    struct timeval tbegin,tend;
    double texec = 0.0;

    double dt = query->time_step;
    unsigned int N = int(query->time_horizon/query->time_step);
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 15; //100;

    #if useILQRSolver
        ILQRSolver::traj lastTraj;
        //=============================================
        // Build wholebody and pass over to kukaArm
        std::string kIiwaUrdf = 
          FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
        std::string schunkPath = 
          FindResourceOrThrow("drake/manipulation/models/wsg_50_description/urdf/wsg_50_mesh_collision_no_world_joint.urdf");
        std::string connectorPath = 
          FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");

        std::string urdf_;
        systems::DiagramBuilder<double> builder;
        multibody::MultibodyPlant<double>* plant_{};
        geometry::SceneGraph<double>* scene_graph_{};
        std::tie(plant_, scene_graph_) =
          multibody::AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
        multibody::Parser parser(plant_, scene_graph_);
        
        const ModelInstanceIndex iiwa_model = 
          parser.AddModelFromFile(kIiwaUrdf, "iiwa");
        const auto& iiwa_base_frame = plant_->GetFrameByName("iiwa_link_0", iiwa_model);
        RigidTransformd X_WI(Eigen::Vector3d(0, 0, 0));
        plant_->WeldFrames(plant_->world_frame(), iiwa_base_frame, X_WI);

        const ModelInstanceIndex conn_model = 
          parser.AddModelFromFile(connectorPath, "connector");
        const auto& iiwa_ee_frame = plant_->GetFrameByName("iiwa_frame_ee", iiwa_model);
        const auto& conn_frame = plant_->GetFrameByName("connector_link", conn_model);
        RigidTransformd X_EC(Eigen::Vector3d(0, 0, 0));
        plant_->WeldFrames(iiwa_ee_frame, conn_frame, X_EC);

        const ModelInstanceIndex wsg_model = 
          parser.AddModelFromFile(schunkPath, "wsg");
        const auto& wsg_frame = plant_->GetFrameByName("wsg_50_base_link", wsg_model);
        RigidTransformd X_EG(RollPitchYaw<double>(0, 0, M_PI_2),
                                    Vector3d(0, 0, 0.0175));
        plant_->WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

        plant_->Finalize();

        auto context_ptr = plant_->CreateDefaultContext();
        auto context = context_ptr.get();

        VectorXd q_v_iiwa(14);
        q_v_iiwa.setZero();
        q_v_iiwa.head(7) = xinit;
        plant_->SetPositionsAndVelocities(context, iiwa_model, q_v_iiwa);

        MatrixXd M_(plant_->num_velocities(), plant_->num_velocities());
        plant_->CalcMassMatrix(*context, &M_);

        VectorXd gtau_wb(plant_->num_velocities());
        plant_->CalcBiasTerm(*context, &gtau_wb);

        cout << "bias total" << endl << gtau_wb << endl;
        commandVecTab_t u_0;
        u_0.resize(N);
        for(unsigned i=0;i<N;i++){
          u_0[i].head(7) = gtau_wb;
        }
        //======================================
        #if WHOLE_BODY
          KukaArm KukaArmModel(dt, N, xgoal, plant_);
        #else
          KukaArm KukaArmModel(dt, N, xgoal);
        #endif
        CostFunctionKukaArm costKukaArm(N);
        ILQRSolver testSolverKukaArm(KukaArmModel,costKukaArm,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverKukaArm.firstInitSolver(xinit, xgoal, u_0, N, dt, iterMax, tolFun, tolGrad);     
    #endif
    #if useUDPSolver
        double scale = 1e-4;//[To be optimized]
        KukaArm KukaArmModel(dt, N, xgoal);
        KukaArm::timeprofile finalTimeProfile;
        UDPSolver::traj lastTraj;
        CostFunctionKukaArm costKukaArm;
        UDPSolver testSolverKukaArm(KukaArmModel,costKukaArm,ENABLE_FULLDDP,ENABLE_QPBOX);
        testSolverKukaArm.firstInitSolver(xinit, xgoal, N, dt, scale, iterMax, tolFun, tolGrad);    
    #endif

    // run one or multiple times and then average
    unsigned int Num_run = 1;
    gettimeofday(&tbegin,NULL);
    for(unsigned int i=0;i<Num_run;i++) testSolverKukaArm.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = testSolverKukaArm.getLastSolvedTrajectory();
    #if useUDPSolver
      finalTimeProfile = KukaArmModel.getFinalTimeProfile();
    #endif
    joint_state_traj.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = lastTraj.xList[i];
    }
    torque_traj = lastTraj.uList;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<stateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
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

    // cout << "\tTime of RBT massMatrix.inverse (second): " << finalTimeProfile.time_period2 << " (" << 100.0*finalTimeProfile.time_period2/texec << "%)" << endl;
    // cout << "\tTime of RBT f_ext instantiation (second): " << finalTimeProfile.time_period3 << " (" << 100.0*finalTimeProfile.time_period3/texec << "%)" << endl;
    // cout << "\tTime of RBT dynamicsBiasTerm (second): " << finalTimeProfile.time_period4 << " (" << 100.0*finalTimeProfile.time_period4/texec << "%)" << endl;
    
    // // debugging trajectory and control outputs (print func)
    // cout << "--------- final joint state trajectory ---------" << endl;
    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    // }
    // cout << "--------- final joint torque trajectory ---------" << endl;
    
    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.uList[" << i << "]:" << lastTraj.uList[i].transpose() << endl;
    // }

    cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList[N].transpose() << endl;
    cout << "lastTraj.uList[" << N << "]:" << lastTraj.uList[N].transpose() << endl;

    cout << "lastTraj.xList[0]:" << lastTraj.xList[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << lastTraj.uList[0].transpose() << endl;

    for(unsigned int i=N-50;i<=N;i++){
      cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    }
    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory");
      saveVector(torque_traj[i], "joint_torque_command");
    }
    saveVector(lastTraj.xList[N], "joint_trajectory");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated");
    }

    cout << "-------- DDP Trajectory Generation Finished! --------" << endl;
    // traj_knot_number_ = 0;

    // Send over points using LCM
    // need this for dynamic memory allocation (push_back)
    auto ptr = std::make_unique<lcmt_ddp_traj>();
    
    ptr->dim_torques = 0;//kNumJoints;
    ptr->dim_states = kNumJoints; //disregard joint velocity
    ptr->n_time_steps = N*InterpolationScale; 
    ptr->cost = lastTraj.finalCost;

//****************************************************************************************************************
//****************************************************************************************************************
// DOES NOT MATTER FOR POSITION CONTROL -- ALSO JUST FOR DEBUGGING TORQUE, 
// EVERYTHING RELATED TO THIS SHOULD BE HANDLED IN "kuka_arm.cpp"
//****************************************************************************************************************
    // //==========================
    // // const char* kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7.urdf";
    // const char* const kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // const char* schunkPath = "drake/manipulation/models/wsg_50_description/urdf/wsg_50_mesh_collision_no_world_joint.urdf";
    // const char* connectorPath = "drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf";
    
    // std::string urdf_;
    // RigidBodyTree<double> kukaTree_;
    // std::unique_ptr<RigidBodyTree<double>> totalTree_;
    // totalTree_ = std::make_unique<RigidBodyTree<double>>();

    // VectorX<double> states;
    // Eigen::VectorXd robot_q;
    // Eigen::VectorXd iiwa_q;
    // Eigen::VectorXd iiwa_v;
    // std::unique_ptr<WorldSimTreeBuilder<double>> tree_builder; 
    // std::unique_ptr<RigidBodyPlant<double>> plant;

    // robot_q = Eigen::VectorXd(9);
    // iiwa_q = Eigen::VectorXd(7);
    // iiwa_v = Eigen::VectorXd(7);

    // tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
    // tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
    // tree_builder->StoreDrakeModel("wsg", schunkPath);
    // tree_builder->StoreDrakeModel("connector", connectorPath);


    // const Eigen::Vector3d kRobotBase(0, 0, 0);


    // ModelInstanceInfo<double> iiwa_instance, connector_instance, wsg_instance;


    // int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
    // iiwa_instance = tree_builder->get_model_info_for_instance(id);


    // id = tree_builder->AddModelInstanceToFrame(
    //     "connector", tree_builder->tree().findFrame("iiwa_frame_ee"),
    //     drake::multibody::joints::kFixed);
    // connector_instance = tree_builder->get_model_info_for_instance(id);
    

    // //The schunk will be 0.09 above the kuka end effector -> change number by going to the urdf of iiwa7
    // id = tree_builder->AddModelInstanceToFrame(
    //     "wsg", tree_builder->tree().findFrame("connector_frame"),
    //     drake::multibody::joints::kFixed);
    // wsg_instance = tree_builder->get_model_info_for_instance(id);

    // totalTree_ = tree_builder->Build();

    // VectorXd q(9);
    // VectorXd qd(9);
    //   // VectorX<double> q = VectorX<double>::Ones(7);
    //   // VectorX<double> qd = VectorX<double>::Zero(7);
    // //  q << 0,0.602162,0.00180032,-1.75906,-0.0181392,0.962099,0.0121962;
    // // q << 1,1,1,1,1,1,1,0,0;
    // q << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0, 0;
    // // q << 0,0.5,0,0.5,0,0,0;
    // qd << 0,0,0,0,0,0,0, 0,0;

    // // std::unique_ptr<RigidBodyTree<double>> robot_thread_;
    // // robot_thread_ = std::make_unique<RigidBodyTree<double>>();
    // // const char* const kIiwaUrdf =
    // //         "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // // parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    // //         FindResourceOrThrow(kIiwaUrdf),
    // //         multibody::joints::kFixed, robot_thread_.get());

    // KinematicsCache<double> cache_ = totalTree_->CreateKinematicsCache();
    // cache_.initialize(q,qd);
    // totalTree_->doKinematics(cache_, true);

    // MatrixX<double> M_ = totalTree_->massMatrix(cache_); // Inertial matrix

    // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

    // // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
    // VectorX<double> gtau_wb = totalTree_->inverseDynamics(cache_, f_ext, qd, false);

    // cout << "bias total" << endl << gtau_wb << endl;

    // //=======================
    // // Find Arm Gravity Compensation
    // VectorXd q_a(7);
    // VectorXd qd_a(7);
    // // q_a << 1,1,1,1,1,1,1;
    // q_a << 0, 0.6, 0, -1.75, 0, 1.0, 0;
    // qd_a << 0,0,0,0,0,0,0;

    // std::unique_ptr<RigidBodyTree<double>> robot_thread_;
    // robot_thread_ = std::make_unique<RigidBodyTree<double>>();
    // const char* const kIiwaUrdf_a =
    //         "drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf";
    // parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    //         FindResourceOrThrow(kIiwaUrdf_a),
    //         multibody::joints::kFixed, robot_thread_.get());

    // KinematicsCache<double> cache_a_ = robot_thread_->CreateKinematicsCache();
    // cache_a_.initialize(q_a,qd_a);
    // robot_thread_->doKinematics(cache_a_, true);

    // MatrixX<double> M_a = robot_thread_->massMatrix(cache_a_); // Inertial matrix

    // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext_a;

    // // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
    // VectorX<double> gtau = robot_thread_->inverseDynamics(cache_a_, f_ext_a, qd_a, false);

    // cout << "bias arm" << endl << gtau << endl;

    // // subtract biases
    // for (int k=0; k<7; k++) {
    //   gtau[k] = gtau_wb[k] - gtau[k];
    // }

    // cout << "bias subtracted" << endl << gtau << endl;

      //============================================
//****************************************************************************************************************
//****************************************************************************************************************


    for (int32_t i=0; i < ptr->n_time_steps; ++i) {
      // need new, cuz dynamic allocation or pointer
      ptr->times_sec.push_back(static_cast<double>(query->time_step*i/InterpolationScale));
      // cout << query->time_step*i/InterpolationScale << endl;

      // cout << ptr->times_sec[i] << endl;

      auto ptr2 = std::make_unique<std::vector<double>>();
      auto ptr2_st = std::make_unique<std::vector<double>>();

      for (int32_t j=0; j < ptr->dim_states; ++j) { 
        //  ptr2->push_back(lastTraj.uList[i][j]);
        // ptr2->push_back(gtau[j]);

        ptr2->push_back(0);
        ptr2_st->push_back(joint_state_traj_interp[i][j]);
      }
      ptr->torques.push_back(*ptr2);
      ptr->states.push_back(*ptr2_st);
    }

    // cout << "check1" << endl;

    // // need this because...?
    // ddp_traj_ = *ptr;

    // cout << "check2" << endl;

    // cout << ddp_traj_.dim_torques << endl;
    // cout << ddp_traj_.dim_states << endl;
    // cout << ddp_traj_.n_time_steps << endl;
    // cout << ddp_traj_.cost << endl;

    // //sanity check
  //     for (int32_t i=0; i<ddp_traj_.n_time_steps; ++i) {
  ////       cout << ddp_traj_.times_sec[i] << endl;
  //       for (int32_t j=0; j<ddp_traj_.dim_torques; ++j) {
  //         cout << "torq? " << ddp_traj_.torques[i][j] << endl;
  //       }
  //     }

    // lcm_.publish(kLcmQueryResultsChannel, &ddp_traj_);
    // cout << "-------- DDP Trajectory Published to LCM! --------" << endl;
    return *ptr;
}

void DDPRunner::saveVector(const Eigen::MatrixXd & _vec, const char * _name) {
      std::string _file_name = UDP_TRAJ_DIR;
      _file_name += _name;
      _file_name += ".csv";
      DDPRunner::clean_file(_name, _file_name);

      std::ofstream save_file;
      save_file.open(_file_name, std::fstream::app);
      for (int i(0); i < _vec.rows(); ++i){
          save_file<<_vec(i,0)<< "\t";
      }
      save_file<<"\n";
      save_file.flush();
      save_file.close();
}

void DDPRunner::saveValue(double _value, const char * _name){
    std::string _file_name = UDP_TRAJ_DIR;
    _file_name += _name;
    _file_name += ".csv";
    DDPRunner::clean_file(_name, _file_name);

    std::ofstream save_file;
    save_file.open(_file_name, std::fstream::app);
    save_file<<_value <<"\n";
    save_file.flush();
}

void DDPRunner::clean_file(const char * _file_name, std::string & _ret_file){
    std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if(gs_fileName_string.end() == iter){
        gs_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

} //namespace kuka_iiwa_arm
} //namespace traj_gen
} //namespace drake