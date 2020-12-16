#include <cstdio>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/constraint.h"
#include "drake/common/find_resource.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/traj_gen/constraint/fk_constraint.h"

using namespace Eigen;
using namespace std;

namespace drake{

using multibody::MultibodyPlant;
using multibody::Parser;
using math::RigidTransformd;
using multibody::MultibodyForces;
using multibody::ModelInstanceIndex;
using math::RollPitchYaw;
using solvers::Constraint;
using traj_gen::FKConstraint;

class CollisionAvoidance_ADMM{
    public:
    CollisionAvoidance_ADMM(const drake::multibody::MultibodyPlant<double>& plant): plant_(plant){}
    ~CollisionAvoidance_ADMM(){};

    void SolveOpt(){
        // Define the optimization problem
        solvers::MathematicalProgram prog = solvers::MathematicalProgram();

        // Instantiate the decision variables
        auto x = prog.NewContinuousVariables(7, 5, "x");
        auto x0 = MatrixXd(7, 5);
        x0.setZero();
        Vector1d lb, ub;
        lb << 0.05;
        ub << 100;
        Vector3d target;
        target << 0.1, 0.1, 0.1;

        for(unsigned int i=0;i<5;i++){
            auto x_var = x.col(i);
            auto cost2 = prog.AddL2NormCost(MatrixXd::Identity(7,7), VectorXd::Zero(7), x_var);
            prog.AddConstraint(make_shared<drake::traj_gen::FKConstraint<double>>(plant_, target, "iiwa", "iiwa_link_ee_kuka", 
                                            lb, ub, "FK"), x_var);
        }

        prog.SetInitialGuess(x, x0);
        auto result = solvers::Solve(prog);
        cout << "Is optimization successful?" << result.is_success() << endl;
        cout << "Optimal x: " << result.GetSolution() << endl;
        cout << "solver is: " << result.get_solver_id().name() << endl;
    }

    VectorXd FK(const VectorXd& q){
        auto context_ptr = plant_.CreateDefaultContext();
        auto context = context_ptr.get();

        auto iiwa_model = plant_.GetModelInstanceByName("iiwa");
        auto wsg_model = plant_.GetModelInstanceByName("wsg");
        plant_.SetPositions(context, iiwa_model, q);
        // auto p_EE = plant_.CalcRelativeTransform(*context, plant_.world_frame(), plant_.GetFrameByName("iiwa_link_ee_kuka", iiwa_model));
        auto p_EE = plant_.CalcRelativeTransform(*context, plant_.world_frame(), plant_.GetFrameByName("right_ball_contact3", wsg_model));
        return p_EE.translation();
    }

    private:
    const drake::multibody::MultibodyPlant<double>& plant_;

};

void do_main(){
    // Import KUKA model
    std::string kIiwaUrdf = 
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
    std::string schunkPath = 
        FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf");
    std::string connectorPath = 
        FindResourceOrThrow("drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf");

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
    RigidTransformd X_EG(RollPitchYaw<double>(0, 0, M_PI_2),
                                Vector3d(0, 0, 0.0175));
    plant_.WeldFrames(iiwa_ee_frame, wsg_frame, X_EG);

    plant_.Finalize();
    CollisionAvoidance_ADMM ca(plant_);
    ca.SolveOpt();
    VectorXd q0(7);
    q0.setZero();
    cout << ca.FK(q0) << endl;
}
}
int main(){
    drake::do_main();
    return 0;
}