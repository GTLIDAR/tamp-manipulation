#!/usr/bin/env python3

import time
import os
import sys
from datetime import datetime
import copy
import json
import numpy as np
from multiprocessing import Process
import select

from lcm import LCM
from drake import (lcmt_combined_object_state, lcmt_iiwa_status, lcmt_schunk_wsg_status,
    lcmt_manipulator_traj, lcmt_generic_string_msg)

pddl_path = "/home/zhigen/code/pddl_planning"
drake_path = "/home/zhigen/code/drake"

if pddl_path not in sys.path:
    sys.path.append(pddl_path)

from causal_graph.tools import build_causal_graph, get_subproblems, generate_subtask
from search_tree.tamp_node import PddlTampNode
from search_tree.tamp_tree import PddlTree
from search_tree.multi_wp_motion_plan_runner import ReactiveMultiWPStaionaryManipMotionPlanRunner

from pyperplan import _parse, _ground

TABLE_X = 0.7112
TABLE_Y = 0.762
TABLE_Z = 0.7645
N_OBJECT = 8
USE_TORQUE = False

class StationaryObjectSortingReactivePlanner:
    """ A reactive planner based on causal graph subtast separation

    Attributes:

    """
    def __init__(self, task, geo_setup_file, traj_setup_file):
        self.task_orig = task
        self.task_cur = task
        self.causal_graph = build_causal_graph(task, show=False)
        self.state = self.task_orig.initial_state
        self.subproblems = []
        self.geo_setup_file = geo_setup_file
        self.traj_setup_file = traj_setup_file

        self.plan_trees = []
        self.planned_state = task.initial_state
        self.planned_manipulator_pos = None
        self.planned_time = 0

        self.execute_queue = []
        self.executed_actions = []
        self.executed_trajectories = []

        self.plan_horizon = 1

        self._object_state = None
        self._iiwa_status = None
        self._wsg_status = None

        self._is_sim_started = False
        self._executing_tree = False
        self._planning = False
        self._replan = False

        self._lcm = LCM()
        self._lcm.subscribe("OBJECT_STATE", self._object_state_handler)
        self._lcm.subscribe("IIWA_STATUS", self._iiwa_status_handler)
        self._lcm.subscribe("SCHUNK_WSG_STATUS", self._wsg_status_handler)
        self._lcm.subscribe("START_PLAN", self._start_plan_handler)

################################ lcm handlers ##################################
    def _object_state_handler(self, channel, msg):
        self._object_state = lcmt_combined_object_state.decode(msg)
        self.motion_plan_runner.update_object_state(self._object_state)

    def _iiwa_status_handler(self, channel, msg):
        self._iiwa_status = lcmt_iiwa_status.decode(msg)

    def _wsg_status_handler(self, channel, msg):
        self._wsg_status = lcmt_schunk_wsg_status.decode(msg)

    def _start_plan_handler(self, channel, msg):
        print("Simulation started")
        self._is_sim_started = True    

###############################sense object state################################
    def _update_object_location(self, object_name):
        object_id = int(object_name[-1])
        (x, y, z, r, p, y) = self._object_state.q[object_id]
        
        object_loc = ""
        if (1.5*TABLE_X >= x >= 0.5*TABLE_X) and (0.5*TABLE_Y >= y >= -0.5*TABLE_Y):
            object_loc = "(on " + object_name + " table_0)"
        elif (0.5*TABLE_X >= x >= -0.5*TABLE_X) and (1.5*TABLE_Y >= y >= 0.2*TABLE_Y):
            object_loc = "(on " + object_name + " table_1)"
        elif (0.5*TABLE_X >= x >= -0.5*TABLE_X) and (-0.2*TABLE_Y >= y >= -1.5*TABLE_Y):
            object_loc = "(on " + object_name + " table_2)"
        
        if object_loc == "":
            print("Object Not On Table!")
            return None
        else:
            print(object_loc)
            return object_loc

    def _check_obstruction(self, obj_1, obj_2):
        return False

    def _update_object_predicates(self):
        object_name_list = []
        for i in range(N_OBJECT):
            object_name_list.append("box_" + str(i))
        
        remove_list = []
        add_list = []

        for ob in object_name_list:
            cur_obj_loc = self._update_object_location(ob)
            
            if cur_obj_loc is not None:
                add_list.append(cur_obj_loc)
                for pre in self.state:
                    if pre.startswith(cur_obj_loc[:9]):
                        remove_list.append(pre)

        for ob1 in object_name_list:
            for ob2 in object_name_list:
                if not self._check_obstruction(ob1, ob2):
                    add_list.add("(unobstructed "+ob1+" "+ob2+")")
                elif ("(unobstructed "+ob1+" "+ob2+")") in self.state:
                    remove_list.add("(unobstructed "+ob1+" "+ob2+")")
        
        changed = False
        for pre in remove_list:
            if pre in self.state:
                self.state.remove(pre)
                changed = True

        for pre in add_list:
            if pre not in self.state:
                self.state.append(pre)
                changed = True
        
        self.task_cur.initial_state = self.state

        return changed

    def _update_subproblems(self):
        self.subproblems = get_subproblems(self.causal_graph, self.task_cur)
    
    def _rank_subproblems(self):
        # TODO: Heuristics to rank subproblems
        pass

###############################solve holistic###################################
    def _plan_subproblem(self, subproblem, init_state):
        self._planning = True
        subtask = generate_subtask(self.task_orig, subproblem, init_state)

        print("Planning new subtask")
        print(subtask)

        #FIXME: Initialize Continuous state of root node, currently only init
        # to all zero
        root = PddlTampNode.make_root_node(init_state)
        if self.planned_manipulator_pos is not None:
            root.traj = self.planned_manipulator_pos
            root.time = self.planned_time

        start = time.time()
        tree = PddlTree(
            root, 
            subtask, 
            ReactiveMultiWPStaionaryManipMotionPlanRunner(
                self.geo_setup_file, self.traj_setup_file
            )
        )

        tree.motion_plan_runner.update_object_state(self._object_state)

        (tree.goals, n_visited) = tree.hybrid_search(
            total_depth_limit=15, n_sols=1
        )

        print("Subtask search time:", time.time()-start)

        if len(tree.goals):
            self.plan_trees.append(tree)
            actions = tree.get_sol(tree.goals[0])
            for act in actions:
                self.planned_state = act.apply(self.planned_state)
            self.planned_manipulator_pos = tree.goals[0].traj
            self.planned_time = tree.goals[0].time
            print("Tree Added.")
        else:
            print("This tree does not have solution...")
            print(subtask)
            input("Press Enter to exit")
        
        self._planning = False

###############################execute plan#####################################
    def _execute_tree(self, tree):
        """ Add tree nodes to execution queue
        """
        self._executing_tree = True
        actions = tree.get_sol(tree.goals[0])
        trajectories = tree.get_traj(tree.goals[0])

        cur_node_id = 0

        for cur_node_id in range(len(actions)):
            print("Node", cur_node_id)
            print(actions[cur_node_id])
            traj_runner = TrajectoryRunner()
            traj_runner.run_trajectory(trajectories[cur_node_id])
            self.state = actions[cur_node_id].apply(self.state)
            self.executed_actions.append(actions[cur_node_id])
            self.executed_trajectories.append(trajectories[cur_node_id])
        
            if self._update_object_predicates():
                self._update_subproblems()
                self.planned_manipulator_pos = trajectories[cur_node_id]
                print("Something Unexpected Happened.")
                print("Replanning")
                self._replan = True
                break
                
        self._executing_tree = False

###############################main runner######################################
    def run(self):
        # pre planning
        self._update_subproblems()
        self._rank_subproblems()
        
        while len(self.plan_trees) < self.plan_horizon:
            sp = self.subproblems.pop(0)
            self._plan_subproblem(sp, self.planned_state)

        input("Preplanning Completed...")
        
        while not self.task_orig.goal_reached(self.state):
            if len(self.planned_trees) and (not self._executing_tree):
                print("start tree execution process")
                execution_process = Process(
                    target=self._execute_tree, args=(self.planned_trees.pop(0),))
                execution_process.start()
            
            if (len(self.planned_trees)<self.plan_horizon) and (not self._planning) and len(self.subproblems):
                print("start planning process")
                sp = self.subproblems.pop(0)
                planning_process = Process(
                    target=self._plan_subproblem, args=(sp, self._planned_state))
                planning_process.start()
            
            if self._replan:
                if self._planning:
                    planning_process.terminate()
                    self._planning = False
                self.planned_trees = []
                self.planned_state = self.state
                self.planned_time = self._object_state.utime/1e6
                
                while len(self.planned_trees) < self.plan_horizon:
                    sp = self.subproblems.pop()
                    self._plan_subproblem(sp, self.planned_state)


class TrajectoryRunner:
    def __init__(self):
        self._lcm = LCM()
        self._lcm.subscribe("EXECUTION_STATUS", self._kuka_plan_runner_status_handler)

        self.use_torque = False
        self._completed = False

    def _kuka_plan_runner_status_handler(self, channel, msg):
        print("Kuka Traj Completed")
        self._completed = True
    
    def _publish_node_traj(self, node):
        msg = dict_to_lcmt_manipulator_traj(node)

        if USE_TORQUE:
            msg.dim_torques = 7
        else:
            msg.dim_torques = 0

        if msg.n_time_steps:
            self._lcm.publish("COMMITTED_ROBOT_PLAN", msg.encode())
            print("Trajectory Published!")
    
    def run_trajectory(self, node):
        self._publish_node_traj(node)
        try:
            timeout = 5000
            while not self._completed:
                rfds, wfds, efds = select.select(
                    [self._lcm.fileno()], [], [], timeout)
                if rfds:
                    self._lcm.handle()
        except KeyboardInterrupt:
            pass

    

def main():
    domain_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/domain_7obj.pddl"
    problem_file = drake_path + "/conveyor_belt_tamp/pddl/object_sorting/problem_7obj.pddl"
    geo_setup_file = drake_path + "/conveyor_belt_tamp/setup/object_sorting/geo_setup.json"
    traj_setup_file = drake_path + "/conveyor_belt_tamp/setup/object_sorting/traj_setup.json"

    problem = _parse(domain_file, problem_file)
    task = _ground(problem)

    planner = StationaryObjectSortingReactivePlanner(task, geo_setup_file, traj_setup_file)

    planner.run()

if __name__=="__main__":
    main()








