import json
import numpy as np

from lcm import LCM
from drake import lcmt_multi_wp_manip_query

from search_tree.query import MultiWPConveyorBeltManipQuery, StationaryMultiWPManipQuery, StationaryMultiWPManipReactiveQuery

from search_tree.motion_plan_runner import BasicManipMotionPlanRunner

class MultiWPConveyorBeltManipMotionPlanRunner(BasicManipMotionPlanRunner):
    def __init__(self, geo_setup_file, traj_setup_file):
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._results_handler)
        self._query = MultiWPConveyorBeltManipQuery(geo_setup_file)
        self._node = None
        self._traj_perturbed = False

        with open(traj_setup_file) as fp:
            self._traj_setup = json.load(fp)

    def _generate_move_query(self, node, op_name):
        """ Generates Multi WP move query for traj op

        Args:
            node: tree search node
            op_name: (string) name of operation
        
        Returns:
            msg: (lcmt_multi_wp_manip_query)
        """
        if op_name.startswith("throw"):
            print("Generating Throw Action Query")
            node.time = node.parent.time + sum(self._traj_setup[op_name]["time_horizon"])
            msg = lcmt_multi_wp_manip_query()
            msg.name = node.action.name[1:-1].split(" ")[0]
            msg.option = node.option
            msg.bypass_ik = True
            msg.dim_q = 7
            msg.n_wp = 2
            msg.time_horizon = self._traj_setup[op_name]["time_horizon"]
            msg.time_step = self._traj_setup["ddp_time_step"]
            msg.desired_ee = [[0 for _ in range(6)] for _ in range(msg.n_wp)]
            msg.q_goal = [self._traj_setup[op_name]["init_config"],
                            self._traj_setup[op_name]["goal_config"]]
            if node.parent_traj:
                msg.prev_q = node.parent_traj["states"][-1]
                msg.prev_gripper_width = node.parent_traj["gripper_width"][-1]
            else:
                msg.prev_q = self._traj_setup["init_iiwa_pos"]
                msg.prev_gripper_width = self._traj_setup["gripper_open_width"]
            msg.gripper_force = self._traj_setup["gripper_force"]
            msg.constrain_orientation = [True, True]
        
        else:
        
            if node.parent_traj:
                q_prev = node.parent_traj["states"][-1]
                gripper_width_prev = node.parent_traj["gripper_width"][-1]
            else:
                q_prev = self._traj_setup["init_iiwa_pos"]
                gripper_width_prev = self._traj_setup["gripper_open_width"]
            
            if node.parent is None:
                print("q_prev:", q_prev)
                input("")

            print("Parent EE:", node.parent.final_ee)
            if (node.parent.final_ee is not None) and (
                "init_offset_wp" in self._traj_setup[op_name]):
                # self._traj_setup[op_name]["init_offset_wp"] is not None):
                print("Adding lift waypoint")
                print("Lift:", self._traj_setup[op_name]["init_offset_wp"])
                wp = (np.array(node.parent.final_ee) 
                    + np.array(self._traj_setup[op_name]["init_offset_wp"])).tolist()
                ee_desired_list = [wp]
                time_horizon_list = [self._traj_setup[op_name]["init_offset_time"]]
                constrain_orientation_list = [self._traj_setup[op_name]["init_offset_constrain_orientation"]]
            else:
                ee_desired_list = []
                time_horizon_list = []
                constrain_orientation_list = []


            time_horizon_list.extend(self._traj_setup[op_name]["time_horizon"])
            node.time = node.parent.time + sum(time_horizon_list)
            ee_desired_list.extend(self._query.get_desired_ee_pos(node))
            node.final_ee = ee_desired_list[-1]
            constrain_orientation_list.extend(self._traj_setup[op_name]["constrain_orientation"])
            print("Node Time:", node.time)
            print("Prev q:", q_prev)
            print("DESIRED EE:", ee_desired_list)

            msg = lcmt_multi_wp_manip_query()

            msg.name = node.action.name[1:-1].split(" ")[0]
            msg.dim_q = 7
            msg.n_wp = len(ee_desired_list)
            msg.option = node.option

            op = node.action.name[1:-1]
            if op.split(" ")[0] == "move-to-push":
                msg.wait_time = self._query.get_push_wait_time(node)
                print("adding push wait time:", msg.wait_time)
                node.time += msg.wait_time

            else:
                msg.wait_time = 0

    ################################################################################
            # Setup false bypass ik
            msg.bypass_ik = False
            msg.q_goal = []
            for i in range(msg.n_wp):
                msg.q_goal.append([0 for j in range(msg.dim_q)])
    ################################################################################

            if node.option=="ddp":
                msg.time_step = self._traj_setup["ddp_time_step"]
            elif node.option=="admm":
                msg.time_step = self._traj_setup["admm_time_step"]

            msg.time_horizon = time_horizon_list

            msg.prev_gripper_width = gripper_width_prev
            msg.desired_ee = ee_desired_list
            msg.prev_q = q_prev
            msg.gripper_force = self._traj_setup["gripper_force"]
            msg.constrain_orientation = constrain_orientation_list

################################################################################
        # set object states
        op = node.action.name[1:-1]

        if (node.parent.final_ee is not None 
            and (op.split(" ")[0]=="move-to-bin" 
            or op.split(" ")[0]=="push"
            # or op.split(" ")[0]=="throw"
            )):
            object_name = op.split(" ")[2]

            vel_obj = [0 for _ in range(6)]
            xyz_iiwa = [0.05, 0, 0]
            msg.use_object = True
            q_obj_init = self._query.get_q_object_init(object_name, node.parent.time, xyz_iiwa)
            q_obj_list = []
            
            for i in range(msg.n_wp):
                if i == 0:
                    ee_init = node.parent.final_ee
                    q_init = q_obj_init
                else:
                    ee_init = msg.desired_ee[i-1]
                    q_init = q_obj_list[i-1]
                
                ee_final = msg.desired_ee[i]

                q_obj_list.append(self._query.get_q_object_final(q_init, ee_init, ee_final))
            
            msg.q_init_object = []
            msg.q_init_object.extend(q_obj_init)
            msg.q_init_object.extend(vel_obj)

            msg.q_desired_object = []
            for i in range(msg.n_wp):
                q_obj_final = []
                q_obj_final.extend(q_obj_list[i])
                q_obj_final.extend(vel_obj)
                msg.q_desired_object.append(q_obj_final)

            print("q_init_object", msg.q_init_object)
            print("q_desired_object", msg.q_desired_object)
                
        
        else:
            msg.use_object = False
            msg.q_init_object = [0 for _ in range(13)]
            msg.q_desired_object = []
            for i in range(msg.n_wp):
                msg.q_desired_object.append([0 for _ in range(13)])

################################################################################
        return msg

class MultiWPStaionaryManipMotionPlanRunner(BasicManipMotionPlanRunner):
    def __init__(self, geo_setup_file, traj_setup_file):
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._results_handler)
        self._query = StationaryMultiWPManipQuery(geo_setup_file)
        self._node = None
        self._traj_perturbed = False

        with open(traj_setup_file) as fp:
            self._traj_setup = json.load(fp)

    def _generate_move_query(self, node, op_name):
        """ Generates Multi WP move query for traj op

        Args:
            node: tree search node
            op_name: (string) name of operation
        
        Returns:
            msg: (lcmt_multi_wp_manip_query)
        """
        
        if node.parent_traj:
            q_prev = node.parent_traj["states"][-1]
            gripper_width_prev = node.parent_traj["gripper_width"][-1]
        else:
            q_prev = self._traj_setup["init_iiwa_pos"]
            gripper_width_prev = self._traj_setup["gripper_open_width"]
        
        if node.parent is None:
            print("q_prev:", q_prev)
            input("")

        print("Parent EE:", node.parent.final_ee)
        if (node.parent.final_ee is not None) and (
            "init_offset_wp" in self._traj_setup[op_name]):
            # self._traj_setup[op_name]["init_offset_wp"] is not None):
            print("Adding lift waypoint")
            print("Lift:", self._traj_setup[op_name]["init_offset_wp"])
            wp = (np.array(node.parent.final_ee) 
                + np.array(self._traj_setup[op_name]["init_offset_wp"])).tolist()
            ee_desired_list = [wp]
            time_horizon_list = [self._traj_setup[op_name]["init_offset_time"]]
            constrain_orientation_list = [self._traj_setup[op_name]["init_offset_constrain_orientation"]]
        else:
            ee_desired_list = []
            time_horizon_list = []
            constrain_orientation_list = []


        ee_desired_list.extend(self._query.get_desired_ee_pos(node))
        node.final_ee = ee_desired_list[-1]
        time_horizon_list.extend(self._traj_setup[op_name]["time_horizon"])
        node.time = node.parent.time + sum(time_horizon_list)
        constrain_orientation_list.extend(self._traj_setup[op_name]["constrain_orientation"])
        print("Prev q:", q_prev)
        print("DESIRED EE:", ee_desired_list)

        msg = lcmt_multi_wp_manip_query()

        msg.name = node.action.name[1:-1].split(" ")[0]
        msg.dim_q = 7
        msg.n_wp = len(ee_desired_list)
        msg.option = node.option

################################################################################
        # Setup false bypass ik
        msg.bypass_ik = False
        msg.q_goal = []
        for i in range(msg.n_wp):
            msg.q_goal.append([0 for j in range(msg.dim_q)])
################################################################################

        if node.option=="ddp":
            msg.time_step = self._traj_setup["ddp_time_step"]
        elif node.option=="admm":
            msg.time_step = self._traj_setup["admm_time_step"]

        msg.time_horizon = time_horizon_list

        msg.prev_gripper_width = gripper_width_prev
        msg.desired_ee = ee_desired_list
        msg.prev_q = q_prev
        msg.gripper_force = self._traj_setup["gripper_force"]
        msg.constrain_orientation = constrain_orientation_list

################################################################################
        # set object states
        op = node.action.name[1:-1]

        if (node.parent.final_ee is not None 
            and (op.split(" ")[0]=="move-to-free-table" 
            or op.split(" ")[0]=="move-to-goal-table")):
            object_name = op.split(" ")[2]

            vel_obj = [0 for _ in range(6)]

            xyz_iiwa = [-0.2, 0, 0]
            msg.use_object = True
            q_obj_init = self._query.get_q_object_init(object_name, node.parent.time, xyz_iiwa)
            q_obj_list = []
            
            for i in range(msg.n_wp):
                if i == 0:
                    ee_init = node.parent.final_ee
                    q_init = q_obj_init
                else:
                    ee_init = msg.desired_ee[i-1]
                    q_init = q_obj_list[i-1]
                
                ee_final = msg.desired_ee[i]

                q_obj_list.append(self._query.get_q_object_final(q_init, ee_init, ee_final))
            
            msg.q_init_object = []
            msg.q_init_object.extend(q_obj_init)
            msg.q_init_object.extend(vel_obj)

            msg.q_desired_object = []
            for i in range(msg.n_wp):
                q_obj_final = []
                q_obj_final.extend(q_obj_list[i])
                q_obj_final.extend(vel_obj)
                msg.q_desired_object.append(q_obj_final)

            print("q_init_object", msg.q_init_object)
            print("q_desired_object", msg.q_desired_object)
                
        
        else:
            msg.use_object = False
            msg.q_init_object = [0 for _ in range(13)]
            msg.q_desired_object = []
            for i in range(msg.n_wp):
                msg.q_desired_object.append([0 for _ in range(13)])

################################################################################
        return msg

class ReactiveMultiWPStaionaryManipMotionPlanRunner(MultiWPStaionaryManipMotionPlanRunner):
    def __init__(self, geo_setup_file, traj_setup_file):
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._results_handler)
        self._query = StationaryMultiWPManipReactiveQuery(geo_setup_file)
        self._node = None
        self._traj_perturbed = False

        with open(traj_setup_file) as fp:
            self._traj_setup = json.load(fp)
            
    def update_object_state(self, object_state):
        self._query.update_object_state(object_state)
