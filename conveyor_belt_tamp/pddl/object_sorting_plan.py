#!/usr/bin/env python

import time
from pathlib import Path
from datetime import datetime
import copy
import json
import numpy as np

from causal_graph_planner import CausalGraphTampPlanner

drake_path = str(Path(__file__).parent.parent.parent.absolute())


from causal_graph.tools import build_causal_graph, get_subproblems, generate_subtask
from search_tree.tamp_node import PddlTampNode
from search_tree.tamp_tree import PddlTree
from search_tree.motion_plan_runner import ConveyorBeltManipMotionPlanRunner
from search_tree.multi_wp_motion_plan_runner import MultiWPStaionaryManipMotionPlanRunner

from pyperplan import _parse, _ground

TRAJ_OPTION = "refine"
MULTI_WP = True

def main():
    # domain_file = drake_path + "/conveyor_belt_tamp/pddl/throw_domain.pddl"
    # problem_file = drake_path + "/conveyor_belt_tamp/pddl/throw_problem.pddl"
    domain_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/domain_10obj.pddl"
    problem_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/problem_10obj.pddl"
    # domain_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/domain_7obj.pddl"
    # problem_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/problem_7obj.pddl"
    # domain_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/domain.pddl"
    # problem_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/problem.pddl"

    problem = _parse(domain_file, problem_file)
    task = _ground(problem)

    geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/object_sorting/geo_setup.json"
    traj_setup_file = drake_path + "/conveyor_belt_tamp/setup/object_sorting/traj_setup.json"

    motion_planner = MultiWPStaionaryManipMotionPlanRunner(
        geo_setup_file, traj_setup_file
    )

    planner = CausalGraphTampPlanner(task, motion_planner)

    start = time.time()
    planner.plan(option=TRAJ_OPTION)
    end = time.time()
    print("Total Planning Time: ", end - start, "sec")
    
    planner.save_traj()

if __name__=="__main__":
    main()
