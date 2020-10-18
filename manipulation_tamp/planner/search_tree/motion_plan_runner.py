import select
import json
import numpy as np
import math
import random

from lcm import LCM
from drake import lcmt_motion_plan_query, lcmt_manipulator_traj

from search_tree.query import ConveyorBeltManipQuery, ConveyorBeltManipReactiveQuery
from utils.traj_utils import lcmt_manipulator_traj_to_dict, dict_to_lcmt_manipulator_traj

class BasicManipMotionPlanRunner(object):
    """ Runs Motion Plan for conveyor belt domain
    """
    def __init__(self, geo_setup_file, traj_setup_file):
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._results_handler)
        self._query = ConveyorBeltManipQuery(geo_setup_file)
        self._node = None
        self._traj_perturbed = False

        with open(traj_setup_file) as fp:
            self._traj_setup = json.load(fp)

    def _results_handler(self, channel, msg):
        print("message received.")
        data = lcmt_manipulator_traj.decode(msg)
        print("Cost:", data.cost)

        if not data.n_time_steps:
            print("IK Infeasible")
        elif data.cost==float('inf') or math.isnan(data.cost) or data.cost==0:
            if self._node.move_query.time_step/2 >= self._traj_setup["min_time_step"]:
                self._node.move_query.time_step /= 2
                self._lcm.publish("TREE_SEARCH_QUERY", self._node.move_query.encode())
                print("Decreasing time step to",
                    self._node.move_query.time_step)
                return
                # ans = input("Trajectory not found. Try Again? [yes/no]")
                # while not (ans == "yes" or ans == "no"):
                #     ans = input("Please input yes or no...")

                # if ans =="yes":
                #     self._lcm.publish("TREE_SEARCH_QUERY", self._node.move_query.encode())
                #     print("Decreasing time step to",
                #         self._node.move_query.time_step)
                #     return
                # elif ans == "no":
                #     print("Traj Op terminated by user. Trajectory cannot be found.") 

            elif not self._traj_perturbed:
                self._node.move_query.time_step = self._traj_setup["admm_time_step"]
                self._perturb_desired_ee(self._node, sigma=0.005)
                self._lcm.publish("TREE_SEARCH_QUERY", self._node.move_query.encode())
                print("Perturbed desired ee, reset time step")
                return
                
            else:
                print("Minimum time step reached. Trajectory cannot be found.")

        self.results = lcmt_manipulator_traj_to_dict(data)
        self.completed = True

    def run(self, node):
        """ returns results of query for conveyor belt manipulation domain

        Args:
            node: (PddlTampNode)
        Returns:
            results: (Dict) containing trajectory info
        """
        self._node = node
        op_name = node.action.name[1:-1].split(" ")[0]
        obj_name = node.action.name[1:-1].split(" ")[2]

        if node.parent is not None:
            node.processed_object = node.parent.processed_object.copy()
            node.discrete_cost = node.parent.discrete_cost
        
        if obj_name not in node.processed_object:
            node.processed_object.append(obj_name)

            if "base_object_cost" in self._traj_setup:
                obj_name = node.action.name[1:-1].split(" ")[2]
                alpha = self._traj_setup["base_object_cost"]["alpha"]
                base_obj_cost = self._traj_setup["base_object_cost"][obj_name]
                discrete_obj_cost = base_obj_cost * (alpha ** len(node.processed_object))
                node.discrete_cost += discrete_obj_cost

                print("Adding New discrete cost", obj_name, discrete_obj_cost)
                print("Total discrete cost", node.discrete_cost)

        self.completed = False

        if self._traj_setup[op_name]["traj_op"]:
            if node.move_query is None:
                node.move_query = self._generate_move_query(node, op_name)
            self._lcm.publish("TREE_SEARCH_QUERY", node.move_query.encode())
            print("LCM Query Published...")
        else:
            if isinstance(self._traj_setup[op_name]["time_horizon"], list):
                node.time = node.parent.time + sum(self._traj_setup[op_name]["time_horizon"])
            else:
                node.time = node.parent.time + self._traj_setup[op_name]["time_horizon"]
            self.results = self._generate_traj(node, op_name)
            self.completed = True


        timeout = 5000
        while (True):
            if self.completed:
                return self.results

            rfds, wfds, efds = select.select(
                [self._lcm.fileno()], [], [], timeout)
            if rfds:
                self._lcm.handle()

    def _perturb_desired_ee(self, node, sigma=0.01):
        if node.move_query is None:
            print("Warning: no move query available")
            return
        
        if isinstance(node.move_query.desired_ee, list):
            for ee in node.move_query.desired_ee:
                for element in ee:
                    element += random.gauss(0, sigma)
        
        else:
            for element in node.move_query.desired_ee:
                element += random.gauss(0, sigma)
        
        self._traj_perturbed = True


    def _generate_move_query(self, node, op_name):
        if node.parent_traj:
            q_prev = node.parent_traj["states"][-1]
            gripper_width_prev = node.parent_traj["gripper_width"][-1]
        else:
            q_prev = self._traj_setup["init_iiwa_pos"]
            gripper_width_prev = self._traj_setup["gripper_open_width"]
        
        if node.parent is None:
            print("q_prev:", q_prev)
            input("")

        ee_desired = self._query.get_desired_ee_pos(node)
        print("Prev q:", q_prev)
        print("DESIRED EE:", ee_desired)

        msg = lcmt_motion_plan_query()
        msg.name = node.action.name[1:-1].split(" ")[0]
        msg.dim_q = 7

        if node.option=="ddp":
            msg.level = 1
            msg.time_step = self._traj_setup["ddp_time_step"]
        elif node.option=="admm":
            msg.level = 2
            msg.time_step = self._traj_setup["admm_time_step"]

        msg.wait_time = self._traj_setup[op_name]["wait_time"]
        msg.time_horizon = self._traj_setup[op_name]["time_horizon"]
        msg.prev_gripper_width = gripper_width_prev

        msg.desired_ee = ee_desired
        msg.prev_q = q_prev
        msg.gripper_force = self._traj_setup["gripper_force"]
        node.final_ee = ee_desired

        return msg

    def _generate_traj(self, node, op_name):
        if isinstance(self._traj_setup[op_name]["time_horizon"], list):
            time_horizon = sum(self._traj_setup[op_name]["time_horizon"])
        else:
            time_horizon = self._traj_setup[op_name]["time_horizon"]
        
        node.final_ee = node.parent.final_ee

        time_step = self._traj_setup["manual_time_step"]

        if self._traj_setup[op_name]["gripper_open"] < 0:
            gripper_width = self._traj_setup["gripper_close_width"]
        elif self._traj_setup[op_name]["gripper_open"] > 0:
            gripper_width = self._traj_setup["gripper_open_width"]
        else:
            if node.parent_traj:
                gripper_width = node.parent_traj["gripper_width"][-1]
            else:
                gripper_width = self._traj_setup["init_gripper_width"]

        msg = dict()
        msg["n_time_steps"] = int(time_horizon/time_step)
        msg["dim_torques"] = 0
        msg["dim_states"] = self._traj_setup["n_joints"]

        msg["cost"] = self._traj_setup[op_name]["cost"]
        cur_time = 0

        msg["times_sec"] = []
        msg["states"] = []
        msg["torques"] = []
        msg["gripper_width"] = []
        msg["gripper_force"] = []

        if node.parent_traj:
            q = node.parent_traj["states"][-1]
        else:
            q = self._traj_setup["init_iiwa_pos"]

        for _ in range(msg["n_time_steps"]):
            msg["states"].append(q)
            msg["torques"].append([])
            msg["gripper_width"].append(gripper_width)
            msg["gripper_force"].append(self._traj_setup["gripper_force"])

            msg["times_sec"].append(cur_time)
            cur_time += time_step

        return msg

class ConveyorBeltManipReactiveMotionPlanRunner(
    BasicManipMotionPlanRunner):
    def __init__(self, geo_setup_file, traj_setup_file):
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._results_handler)
        self._query = ConveyorBeltManipReactiveQuery(geo_setup_file)
        self._node = None

        with open(traj_setup_file) as fp:
            self._traj_setup = json.load(fp)
        
    def update_object_state(self, object_state):
        self._query.update_object_state(object_state)
