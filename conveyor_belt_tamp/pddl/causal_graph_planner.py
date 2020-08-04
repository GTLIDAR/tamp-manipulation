#!/usr/bin/env python

import time
import sys
import os
from datetime import datetime
import copy
import json
import numpy as np

from pydrake.common import FindResourceOrThrow

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

class CausalGraphTampPlanner(object):
    """ A planner based on causal graph subtask separation

    Attributes:
        task: (pyperplan.Task)
        motion_planner: (ConveyorBeltManipMotionPlanRunner)
        causal_graph: (networkx DiGraph) Original causal graph generated from task
        subproblems: a list with (subgoals, ops) pair where "subgoals" is a set
            of subgoals in a disconnected component, and "ops" is a set of operators
            related to the component
        trajectories: (list of nodes) trajectories
        actions: (list of pyperplan.Operation)
        trees: (list of PDDLTrees)
    """
    def __init__(self, task, motion_planner):
        self.task = task
        self.causal_graph = build_causal_graph(task, show=False)
        self.state = task.initial_state
        self.trajectories = []
        self.actions = []
        self.trees = []
        self.motion_planner = motion_planner


    def _rank_subproblems(self, subproblems):
        min_id = []
        for sp in subproblems:
            goals = sp[0]
            id_list = [int(g[g.find("box_")+len("box_")]) for g in goals]
            min_id.append(min(id_list))

        rank = np.argsort(min_id)
        ranked_subproblems = []
        for r in rank:
            ranked_subproblems.append(subproblems[r])
        return ranked_subproblems

    def save_traj(self):
        """ save trajectory
        """
        if not self.trajectories:
            print("No available trajectory.")
            return

        foldername = drake_path + "/conveyor_belt_tamp/results/"
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)

        filename = "traj"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(foldername+filename, 'w') as out:
            json.dump(self.trajectories, out)

    def plan(self, option="ddp"):
        """ Runs plan that uses causal graph to generate subproblems first
        """
        subproblems = get_subproblems(self.causal_graph, self.task, show=True)
        subproblems = self._rank_subproblems(subproblems)
        subtasks = []
        for sp in subproblems:
            subtasks.append(generate_subtask(self.task, sp, self.state))

        root = PddlTampNode.make_root_node(self.state)
        total_n_visited = 0
        start = time.time()
        print("# Subproblem:", len(subtasks))
        for st in subtasks:
            print("Starting new task")
            task_start = time.time()
            tree = PddlTree(root, st, self.motion_planner)
            print(tree.task)
            (tree.goals, n_visited) = tree.hybrid_search(
                total_depth_limit=15, n_sols=1, option=option)
            total_n_visited += n_visited
            print("Subtask Search Time:", time.time()-task_start)
            if len(tree.goals):
                self.state = tree.goals[0].state
                root = PddlTampNode.make_root_node(self.state)
                root.traj = tree.goals[0].traj
                root.time = tree.goals[0].time

                self.trajectories.extend(tree.get_traj(tree.goals[0]))
                self.actions.extend(tree.get_sol(tree.goals[0]))
                self.trees.append(tree)
            else:
                print("This tree does not have solution...")
                input("Press Enter to exit")
                break

        print("CG Plan Completed")
        print("Total time:", time.time()-start)
        print("Solution:")
        for op in self.actions:
            print(op.name)

        return (self.trajectories, self.actions)


def main():
    # domain_file = drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/domain_coupled.pddl"
    domain_file = drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/domain_coupled_stationary.pddl"
    problem_file = drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/3obj_coupled.pddl"

    problem = _parse(domain_file, problem_file)
    task = _ground(problem)

    if MULTI_WP:
        geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/geo_setup_multi_wp.json"
        traj_setup_file = drake_path + "/conveyor_belt_tamp/setup/traj_setup_multi_wp.json"
        motion_planner = MultiWPConveyorBeltManipMotionPlanRunner(
            geo_setup_file, traj_setup_file
        )
    else:
        # geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/geo_setup.json"
        geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/geo_setup_stationary.json"
        traj_setup_file = drake_path + "/conveyor_belt_tamp/setup/traj_setup.json"
        motion_planner = ConveyorBeltManipMotionPlanRunner(
            geo_setup_file, traj_setup_file
        )


    planner = CausalGraphTampPlanner(task, motion_planner)
    planner.plan(TRAJ_OPTION)
    planner.save_traj()

if __name__=="__main__":
    main()
