#!/usr/bin/env python

import time
import sys
import os
from datetime import datetime
import copy
import json
import numpy as np

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
        self.refinement_time = 0
        self.branching_factor = []
        self.num_nodes = []
        self.move_query_sequence = []


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

    def save_traj(self, traj=None, foldername=None, filename=None):
        """ save trajectory
        """
        if traj is None:
            traj = self.trajectories

        if not traj:
            print("No available trajectory.")
            return

        if foldername is None:
            foldername = drake_path + "/conveyor_belt_tamp/results/"
        
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)

        if filename is None:
            filename = "traj"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(foldername+filename, 'w') as out:
            json.dump(traj, out)

    def save_actions(self, actions=None, foldername=None, filename=None):
        """ save actions
        """

        if not actions:
            print("No available solution.")
            return

        if foldername is None:
            foldername = drake_path + "/conveyor_belt_tamp/results/"
        
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)

        if filename is None:
            filename = "actions"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(foldername+filename, 'w') as out:
            json.dump(actions, out)
    
    def save_move_query_sequence(self, query=None, foldername=None, filename=None):
        """ save move query
        """
        if query is None:
            query = self.move_query_sequence
            
        if not query:
            print("No available solution.")
            return

        if foldername is None:
            foldername = drake_path + "/conveyor_belt_tamp/results/"
        
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)

        if filename is None:
            filename = "admm_queries"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(foldername+filename, 'w') as out:
            json.dump(query, out)

    def save_action_costs(self, costs=None, foldername=None, filename=None):
        """ save actions
        """

        if not costs:
            print("No available solution.")
            return

        if foldername is None:
            foldername = drake_path + "/conveyor_belt_tamp/results/"
        
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)

        if filename is None:
            filename = "actions"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(foldername+filename, 'w') as out:
            json.dump(costs, out)

    def plan(self, total_depth_limit=-1, option="ddp", show=False):
        """ Runs plan that uses causal graph to generate subproblems first
        """
        subproblems = get_subproblems(self.causal_graph, self.task, show=show)
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
                total_depth_limit=total_depth_limit, n_sols=1, option=option)
            total_n_visited += n_visited
            (num_nodes, bf) = tree.get_branching_factor()
            self.num_nodes.append(num_nodes)
            self.branching_factor.append(bf)
            self.refinement_time += tree.refinement_time
            print("Subtask Search Time:", time.time()-task_start)
            if len(tree.goals):
                self.state = tree.goals[0].state
                # root = PddlTampNode.make_root_node(self.state)
                # root.traj = tree.goals[0].traj
                # root.time = tree.goals[0].time
                # root.final_ee = tree.goals[0].parent.move_query.desired_ee[-1]
                # root.placed_object = tree.goals[0].placed_object
                root = self._get_next_root_node(tree.goals[0])

                self.trajectories.extend(tree.get_traj(tree.goals[0]))
                self.actions.extend(tree.get_sol(tree.goals[0]))
                self.move_query_sequence.extend(tree.get_move_query_sequence(tree.goals[0], save_results=False))
                self.trees.append(tree)
            else:
                print("This tree does not have solution...")
                raise RuntimeError

        print("CG Plan Completed")
        print("Total time:", time.time()-start)
        print("Solution:")
        for op in self.actions:
            print(op)

        return (self.trajectories, self.actions)
    
    def plan_multiple(self, n_sols=-1, total_depth_limit=-1, option="ddp", show=True):
        foldername = drake_path + "/conveyor_belt_tamp/results/" + datetime.now().strftime("%Y%m%dT%H%M%S") + "/"
        try:
            os.stat(foldername)
        except:
            os.makedirs(foldername)
        
        fp = open(foldername+"result.csv", "w")
        fp.write("idx, time, cost\n")
        fp.close
        
        
        subproblems = get_subproblems(self.causal_graph, self.task, show=show)
        subproblems = self._rank_subproblems(subproblems)
        subtasks = []
        for sp in subproblems:
            subtasks.append(generate_subtask(self.task, sp, self.state))

        total_n_visited = 0
        n_goals = 0
        start = time.time()
        print("# Subproblem:", len(subtasks))
        root = PddlTampNode.make_root_node(self.state)
        tree = PddlTree(root, subtasks[0], self.motion_planner)
        
        problem_stack = [(tree, [], [], [], 0)] # (search tree, previous traj, previous actions, cost vector subtask idx)

        while len(problem_stack):
            tree, prev_traj, prev_actions, prev_costs, st_idx = problem_stack.pop()
            
            print("Starting new task")
            print(tree.task)
            print("total nodes visited", total_n_visited)
            print("planning time: ", time.time()-start)

            (tree.goals, n_visited) = tree.hybrid_search(
                total_depth_limit=total_depth_limit, n_sols=n_sols, option=option)
            total_n_visited += n_visited
            (num_nodes, bf) = tree.get_branching_factor()
            self.num_nodes.append(num_nodes)
            self.branching_factor.append(bf)
            print("# Goal found in subtask:", len(tree.goals))
            print("Subtask completed", str(st_idx)+"/"+str(len(subtasks)-1))
            if len(tree.goals):
                for g in tree.goals:
                    costs = prev_costs.copy()
                    costs.extend(tree.get_cost_seq(g))
                    traj = prev_traj.copy()
                    traj.extend(tree.get_traj(g))
                    actions = prev_actions.copy()
                    actions.extend(tree.get_sol(g))
                    if (st_idx >= len(subtasks)-1): # goal reached
                        n_goals += 1
                        planning_time = time.time()-start
                        fp = open(foldername+"result.csv", "a")
                        fp.write(str(n_goals)+", "+str(planning_time)+", "+str(costs[-1])+"\n")
                        fp.close()
                        self.save_traj(traj=traj, foldername=foldername, filename="traj"+str(n_goals)+".json")
                        self.save_actions(actions=actions, foldername=foldername, filename="actions"+str(n_goals)+".json")
                        self.save_action_costs(costs=costs, foldername=foldername, filename="costs"+str(n_goals)+".json")
    
                    
                        if n_goals >= n_sols >=0:
                            return

                    else: # goal not reached
                        root = self._get_next_root_node(g)
                        tree = PddlTree(root, subtasks[st_idx+1], self.motion_planner)
                        p = (tree, traj, actions, costs, st_idx+1)
                        problem_stack.append(p)


    def _get_next_root_node(self, prev_goal):
        """ Setup next root for search given previous goal node
        """
        init_state = prev_goal.state
        root = PddlTampNode.make_root_node(init_state)
        root.traj = prev_goal.traj
        root.time = prev_goal.time
        root.final_ee = prev_goal.parent.move_query.desired_ee[-1]
        root.placed_object = prev_goal.placed_object
        root.g = prev_goal.g

        return root

def main():
    domain_file = drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/domain_coupled.pddl"
    # domain_file = drake_path + "/conveyor_belt_tamp/pddl/conveyor_belt_multi_grasp_mode/domain_coupled_stationary.pddl"
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
    planner.plan(option=TRAJ_OPTION)
    planner.save_traj()

if __name__=="__main__":
    main()
