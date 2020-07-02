#!/usr/bin/env python3
from __future__ import print_function
from builtins import input
import sys
import os
import copy
import time
import json
from datetime import datetime

file_dir = os.path.dirname(os.path.abspath(__file__))
downward_utils_path = os.path.relpath("../../../pddl_planning/utils", start=file_dir)
sys.path.append(downward_utils_path)
pddl_graph_path = os.path.relpath("../../../pddl_planning/graph", start=file_dir)
sys.path.append(pddl_graph_path)

from downward_utils import (run_search, run_translate, parse_arguments_list,
                            search_from_pddl, read_plan, get_exitcode_msg)

from sas_parser import parse_sas

from pddl_tree import PDDLTree
from pddl_node import PDDLTreeNode

cg_path=os.path.relpath("../../../pddl_planning/PDDLtoGraph", start=file_dir)
sys.path.append(cg_path)

from cg_planner import CGPlanner

DOMAIN_FILE = "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/domain_coupled.pddl"
PROBLEM_FILE = "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/3obj_coupled.pddl"       
BASE_PROBLEM_FILE = "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/3obj_coupled_base_problem.pddl"
SUB_PROBLEM_FILE= "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/3obj_coupled_sub_problem.pddl"

FD_SEARCH_METHOD = "astar(lmcut())"

GRIPPER_WIDTH = [100, 20]
GRIPPER_FORCE = 80
N_SOLS = 1
DISCRETE_DEBUG = 0
DEPTH_LIMIT = -1
SEARCH_LEVEL = 1


class RunPddlManipulationDomain:
    def __init__(self, domain_file, problem_file, search_method, torque_mode=False):
        try:
            os.remove("output.sas")
        except:
            pass
        self.torque_mode = torque_mode
        self.domain_file = domain_file 
        self.problem_file = problem_file
        argstring = ("./downward " + 
                     domain_file + " " + 
                     problem_file + 
                     " --search " + search_method)
        self.orig_argv = sys.argv
        downward_args = parse_arguments_list(argstring.split())
        self.tree = self.init_pddl_tree(downward_args)
        self.goal_seq = None
        
    def init_pddl_tree(self, downward_args):
        run_translate(downward_args)
        env = parse_sas("output.sas")
        print(env)
        root = PDDLTreeNode(None, env["init_state"])
        return PDDLTree(root, env)
    
    def run_pddl_astar(self, depth_limit=-1, n_sols=1):
        self.tree.ddp_a_star(depth_limit=depth_limit, n_sols=n_sols)

    def run_multi_level_search(self, n_levels=2, depth_limit=-1, n_sols=1):
        self.tree.multi_level_search(n_levels=n_levels, depth_limit=depth_limit, n_sols=n_sols)

    def run_discrete_search(self, depth_limit=-1, n_sols=1):
        self.tree.multi_level_search(n_levels=1, depth_limit=depth_limit, n_sols=n_sols, option="discrete")
    
    def save_traj(self, n_trajs=1):
        if not self.tree.goals:
            print("No available trajectory.")
            return
        
        else:

            foldername = "/results/"+datetime.now().strftime("%Y%m%dT%H%M%S")+"/"
            os.mkdir(file_dir+foldername)

            goal_pq = copy.deepcopy(self.tree.goals)
            print("Goal Num:", len(goal_pq))
            print("Saving:", n_trajs)
            
            for i in range(n_trajs):
                if not len(goal_pq):
                    print("No available trajectory")
                    return
                filename = "traj"+str(i)+".json"
                goal_node = goal_pq.pop()
                nodes = self.tree.get_node_sequence(goal_node)
                data = []
                for node in nodes:
                    node_data = {}
                    if node == self.tree.root:
                        continue
                    node_data["g"] = node.g
                    node_data["h"] = node.h
                    node_data["q"] = node.ddp_traj.states
                    node_data["u"] = node.ddp_traj.torques
                    node_data["times_sec"] = node.ddp_traj.times_sec
                    node_data["gripper_width"] = node.ddp_traj.gripper_width
                    node_data["gripper_force"] = node.ddp_traj.gripper_force
                    node_data["n_node_visited"] = node.n_node_visited

                    node_data["name"] = node.name
                    node_data["is_gripper_open"] = node.is_gripper_open
                    data.append(node_data)
                
                with open(file_dir+foldername+filename, "w") as out:
                    json.dump(data, out)
    
    def save_tree(self):
        self.tree.save_tree_svg("tree.svg")


def main():
    # initialize causal graph planner
    cgplanner = CGPlanner(DOMAIN_FILE, PROBLEM_FILE, BASE_PROBLEM_FILE, SUB_PROBLEM_FILE)
    cgplanner.prune_causal_graph()
    goal_list = cgplanner.select_next_subgoal()
    
    # initialize TAMP planner
    runners = []
    runner_idx = 0
    # loop through goal_list
    for subgoals in goal_list:
        print("Searching subgoal", runner_idx)
        print(subgoals)
        cgplanner.write_sub_problem_pddl(subgoals)
        runners.append(
            RunPddlManipulationDomain(
                DOMAIN_FILE, SUB_PROBLEM_FILE, FD_SEARCH_METHOD
            )
        )

        if runner_idx > 0:
            goal_node = runners[0].tree.goals.peak()
            runners[runner_idx].tree.root.q = goal_node.q
            runners[runner_idx].tree.root.prev_op = goal_node.prev_op
            runners[runner_idx].tree.root.time = goal_node.time
            runners[runner_idx].tree.root.g = goal_node.g

        if DISCRETE_DEBUG:
            runners[runner_idx].run_discrete_search()
        else:
            runners[runner_idx].run_multi_level_search(
                n_levels=SEARCH_LEVEL, depth_limit=DEPTH_LIMIT, n_sols=N_SOLS)
        
        if runner_idx > 0:
            runners[0].tree.attach_tree(goal_node, runners[runner_idx].tree)
        
        runner_idx += 1

    goal_pq = copy.deepcopy(runners[0].tree.goals)
    print("Goal Num:", len(goal_pq))
    i = 0
    while (len(goal_pq)):
        node = goal_pq.pop()
        actions = runners[0].tree.get_action_sequence(node)

        print("Goal", i)
        print("Cost", node.g)
        for a in actions:
            print(a)
        print("")
        i += 1

    if not DISCRETE_DEBUG:
        print("Saving result")
        runners[0].save_traj(n_trajs=N_SOLS)
        



if __name__=="__main__":
    main()