#!/usr/bin/env python

import time
import sys
import os
from datetime import datetime
import copy
import json
import numpy as np

from causal_graph_planner import CausalGraphTampPlanner

pddl_path = "/home/zhigen/code/pddl_planning"
if pddl_path not in sys.path:
    sys.path.append(pddl_path)

drake_path = "/home/zhigen/code/drake"

from causal_graph.tools import build_causal_graph, get_subproblems, generate_subtask
from search_tree.tamp_node import PddlTampNode
from search_tree.tamp_tree import PddlTree
from search_tree.motion_plan_runner import ConveyorBeltManipMotionPlanRunner
from search_tree.multi_wp_motion_plan_runner import MultiWPConveyorBeltManipMotionPlanRunner

from pyperplan import _parse, _ground

TRAJ_OPTION = "ddp"
MULTI_WP = True

def main():
    domain_file_list = [
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/1_obj_domain_coupled.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/2_obj_domain_coupled.pddl",        
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/domain_coupled.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/4_obj_domain_coupled.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/5_obj_domain_coupled.pddl"
    ]

    problem_file_list = [
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/1_obj_problem.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/2_obj_problem.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/3obj_coupled.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/4_obj_problem.pddl",
        drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/5_obj_problem.pddl"
    ]

    geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/geo_setup_multi_wp.json"
    traj_setup_file = drake_path + "/conveyor_belt_tamp/setup/traj_setup_multi_wp.json"

    motion_planner = MultiWPConveyorBeltManipMotionPlanRunner(
        geo_setup_file, traj_setup_file
    )

    save_folder = drake_path + "/conveyor_belt_tamp/results/" + datetime.now().strftime("%Y%m%dT%H%M%S") + "/"

    try:
        os.stat(save_folder)
    except:
        os.makedirs(save_folder)

    fp = open(save_folder+"time_result.csv", "w")
    fp.write("name, time\n")
    fp.close()

    for i in range(len(domain_file_list)):
        problem = _parse(domain_file_list[i], problem_file_list[i])
        task = _ground(problem)

        print("Starting Causal Graph Search for", str(i+1), "objects")
        # causal graph planner
        planner = CausalGraphTampPlanner(task, motion_planner)
        start = time.time()
        try:
            planner.plan((i+1)*5, TRAJ_OPTION, False)
            planning_time = time.time() - start

            fp = open(save_folder+"time_result.csv", "a")
            fp.write(str(i+1)+"_obj, "+str(planning_time)+", "+ str(planner.refinement_time)+"\n")
            fp.close()
            planner.save_traj(save_folder, str(i+1)+"_obj_cg.json")
        except RuntimeError:
            pass

        print("Starting single tree Search for", str(i+1), "objects")
        # single tree planner
        root = PddlTampNode.make_root_node(task.initial_state)
        tree = PddlTree(root, task, motion_planner)

        start = time.time()
        (goals, n_visited) = tree.hybrid_search(
            total_depth_limit=(i+1)*5, n_sols=1, option=TRAJ_OPTION)
        planning_time = time.time() - start
        fp = open(save_folder+"time_result.csv", "a")
        fp.write(str(i+1)+"_obj, "+str(planning_time)+", "+ str(tree.refinement_time)+"\n")
        fp.close()
        tree.save_traj(foldername=save_folder, filename=str(i+1)+"_obj.json")


if __name__=="__main__":
    main()