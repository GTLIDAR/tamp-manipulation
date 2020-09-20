#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

using namespace std;
using namespace Eigen;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using multibody::ModelInstanceIndex;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyForces;
using multibody::JacobianWrtVariable;

class jacobian_test
{
public:
    void Run_test(){
        // Add kuka arm welded by connector and gripper
        std::string kIiwaUrdf = 
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/iiwa7_no_world_joint.urdf");
        std::string schunkPath = 
            FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
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

        // Add the object into MBP
        // const std::string box_sdf_path0 = "drake/manipulation/models/kuka_connector_description/urdf/KukaConnector_no_world_joint.urdf";
        const std::string box_sdf_path0 = "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf";
        // const std::string box_sdf_path0 = "drake/conveyor_belt_tamp/models/boxes/redblock.urdf";
        
        auto rpy = math::RollPitchYawd(Eigen::Vector3d(
            0,
            0,
            0
        ));

        auto xyz = Eigen::Vector3d(
                    0.5,
                    0,
                    0
        );

        math::RigidTransform<double> X_WO(
            math::RotationMatrix<double>(rpy),
            xyz
        );


        const ModelInstanceIndex object_model =
        parser.AddModelFromFile(FindResourceOrThrow(box_sdf_path0), "object");

        // const ModelInstanceIndex object_model =
        // parser.AddModelFromFile(FindResourceOrThrow(box_sdf_path0), "object");
        
        // const ModelInstanceIndex wsg_model = 
        // parser.AddModelFromFile(schunkPath, "wsg");

        // const ModelInstanceIndex iiwa_model = 
        // parser.AddModelFromFile(kIiwaUrdf, "iiwa");

        // const auto indices = plant_.GetBodyIndices(object_model);
        // DRAKE_DEMAND(indices.size() == 1);
        // object_ids_.push_back(indices[0]);

        // object_poses_.push_back(X_WObject);
        // object_model_ids_.push_back(model_index);

        plant_.Finalize();

        auto context_ptr = plant_.CreateDefaultContext();
        auto context = context_ptr.get();

        // VectorXd q_v_iiwa(14);
        // q_v_iiwa.setZero();
        
        VectorXd q_v_arm(plant_.num_velocities(iiwa_model)+plant_.num_positions(iiwa_model));
        q_v_arm.setZero();
        // q_v_arm << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 0, 0, 0, 0, 0, 0, 0;
        // cout << q_v_object << endl;
        // cout << object_model << endl;

        // q_v_iiwa.head(7) = xinit;
        // plant_.SetPositionsAndVelocities(context, iiwa_model, q_v_iiwa);
        cout << "object-pos: " << plant_.num_positions(object_model) << endl;
        cout << "object-vel: " << plant_.num_velocities(object_model) << endl;

        cout << "kuka-arm_with_gripper-pos: " << plant_.num_positions() - plant_.num_positions(object_model) << endl;
        cout << "kuka-arm_with_gripper-vel: " << plant_.num_velocities() - plant_.num_velocities(object_model) << endl;

        // cout << "connector: " << plant_.num_velocities(object_model) << endl;
        // cout << "connector: " << plant_.num_positions(object_model) << endl;
        // cout << "wsg: " << plant_.num_velocities(wsg_model) << endl;
        // cout << "wsg: " << plant_.num_positions(wsg_model) << endl;
        // cout << "iiwa: " << plant_.num_velocities(iiwa_model) << endl;
        // cout << "iiwa: " << plant_.num_positions(iiwa_model) << endl;

        plant_.SetFreeBodyPoseInWorldFrame(context, plant_.GetBodyByName("base_link_cracker", object_model), X_WO);
        // plant_.SetFreeBodyPoseInWorldFrame(context, plant_.GetBodyByName("body", wsg_model), X_WO);
        plant_.SetPositionsAndVelocities(context, iiwa_model, q_v_arm);
        
        // plant_.SetPositionsAndVelocities(context, object_model, q_v_object);

        MatrixXd M_(plant_.num_velocities(), plant_.num_velocities());
        plant_.CalcMassMatrix(*context, &M_);
        
        VectorXd Cv(plant_.num_velocities());
        plant_.CalcBiasTerm(*context, &Cv);
        
        Vector3d contact_point;
        MatrixXd Jac(3, plant_.num_velocities());
        contact_point << 0, 0, 0; 
        // plant_.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_.GetFrameByName("iiwa_frame_ee", iiwa_model), contact_point
        // ,plant_.GetFrameByName("base_link_cracker", object_model), plant_.GetFrameByName("base_link_cracker", object_model), &Jac);
         plant_.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV, plant_.GetFrameByName("left_finger", wsg_model), contact_point
        ,plant_.GetFrameByName("base_link_cracker", object_model), plant_.world_frame(), &Jac);

        // VectorXd gtau_wb = plant_.CalcGravityGeneralizedForces(*context);

        // cout << "bias total" << endl << gtau_wb << endl;
        cout << "Mass matrix " << endl << M_ << endl;
        cout << "Bias term " << endl << Cv << endl;
        cout << "Jacobian " << endl << Jac << endl;
    };
};

int do_main(){
    jacobian_test test;
    test.Run_test();
    return 0;
}
}
}
}

int main(){
    return drake::traj_gen::kuka_iiwa_arm::do_main();
}
