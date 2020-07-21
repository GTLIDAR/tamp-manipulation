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

if pddl_path not in sys.path:
    sys.path.append(pddl_path)

from causal_graph.tools import build_causal_graph, get_subproblems, generate_subtask
from search_tree.tamp_node import PddlTampNode
from search_tree.tamp_tree import PddlTree
from search_tree.motion_plan_runner import ConveyorBeltManipReactiveMotionPlanRunner
from utils.traj_utils import dict_to_lcmt_manipulator_traj, lcmt_manipulator_traj_to_dict

from pyperplan import _parse, _ground

USE_TORQUE = False 

class ConveyorBeltManipReactivePlanner:
    """ A planner based on causal graph subtask separation

    Attributes:
        task_orig: (pyperplan.Task) original task
        task_cur: (pyperplan.Task) current remaining task
        causal_graph: (networkx DiGraph) Original causal graph generated from task
        subproblems: a list with (subgoals, ops) pair where "subgoals" is a set
            of subgoals in a disconnected component, and "ops" is a set of operators
            related to the component
        actions: (list of pyperplan.Operation)
        trees: (list of PDDLTrees)
        motion_plan_runner: (ConveyorBeltManipReactiveMotionPlanRunner)
    """
    def __init__(self, task, geo_setup_file, traj_setup_file):
        self._object_state = None
        self._iiwa_status = None
        self._wsg_status = None

        self._lcm = LCM()
        self._lcm.subscribe("OBJECT_STATE", self._object_state_handler)
        self._lcm.subscribe("IIWA_STATUS", self._iiwa_status_handler)
        self._lcm.subscribe("SCHUNK_WSG_STATUS", self._wsg_status_handler)
        self._lcm.subscribe("START_PLAN", self._start_plan_handler)
        # self._lcm.subscribe("EXECUTION_STATUS", self._kuka_plan_runner_status_handler)

        self.geo_setup_file = geo_setup_file
        self.traj_setup_file = traj_setup_file
        self.motion_plan_runner = ConveyorBeltManipReactiveMotionPlanRunner(
            self.geo_setup_file, self.traj_setup_file
        )
        self.traj_runner = TrajectoryRunner()

        self.task_orig = task
        self.task_cur = task
        self.causal_graph = build_causal_graph(task, show=False)
        self.subproblems = []
        self.trees = []
        self.trajectories = []
        self.unplanned_subproblems = []
        self.executed_actions = []
        self.executed_trajectories = []
        self._executed_state = task.initial_state
        self._planned_state = task.initial_state

        
        self.tree_horizon = 1

        self._node_cur = None

        self._is_sim_started = False
        self._cur_node_completed = True
        self._executing_tree = False
        self._planning = False

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
    
    # def _kuka_plan_runner_status_handler(self, channel, msg):
    #     print("Kuka Traj Completed")
    #     self._cur_node_completed = True

##########################      Planner      ###################################
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

    def _pre_plan(self, tree_horizon):
        self.unplanned_subproblems = self._rank_subproblems(
            get_subproblems(self.causal_graph, self.task_cur))
        
        for _ in range(tree_horizon):
            sp = self.unplanned_subproblems.pop(0)
            state = self._plan_subproblem(sp, self._planned_state)
            if state is not None:
                self._planned_state = state

    def _plan_subproblem(self, subproblem, init_state):
        self._planning = True
        subtask = generate_subtask(self.task_orig, subproblem, init_state)

        print("Planning new subtask")
        print(subtask)

        #FIXME: Initialize Continuous state of root node, currently only init
        # to all zero
        root = PddlTampNode.make_root_node(init_state)

        start = time.time()
        tree = PddlTree(
            root, 
            subtask, 
            ConveyorBeltManipReactiveMotionPlanRunner(
                self.geo_setup_file, self.traj_setup_file
            )
        ) 

        (tree.goals, n_visited) = tree.hybrid_search(
            total_depth_limit=15, n_sols=1
        )

        print("Subtask search time:", time.time()-start)

        if len(tree.goals):
            self.trees.append(tree)
            goal_state = tree.goals[0].state
        else:
            print("This tree does not have solution...")
            print(subtask)
            input("Press Enter to exit")
        
        self._planning = False
        
        return goal_state    

        
##########################      Excecutor    ###################################
    def _execute_tree(self, tree):
        """ Add tree nodes to execution queue
        """
        actions = tree.get_sol(tree.goals[0])
        trajectories = tree.get_traj(tree.goals[0])

        cur_node_id = 0

        # print("Node", cur_node_id)
        # print(actions[cur_node_id])
        # self._publish_node_traj(trajectories[cur_node_id])

        # while True:
        #     self._lcm.handle()
        #     if self._cur_node_completed:
        #         self._executed_state = actions[cur_node_id].apply(self._executed_state)
        #         self.executed_actions.append(actions[cur_node_id])
        #         self.trajectories.append(trajectories[cur_node_id])
        #         # TODO: check if real state matches expected state
        #         # TODO: Flag replan if accidents occurs

        #         self._cur_node_completed = False
        #         cur_node_id += 1
        #         if cur_node_id >= len(actions):
        #             print("Tree Executed")
        #             break

        #     print("Node", cur_node_id)
        #     print(actions[cur_node_id])
        #     self._publish_node_traj(trajectories[cur_node_id])
        for cur_node_id in range(len(actions)):
            print("Node", cur_node_id)
            print(actions[cur_node_id])
            self.traj_runner.run_trajectory(trajectories[cur_node_id])
            self._executed_state = actions[cur_node_id].apply(self._executed_state)
            self.executed_actions.append(actions[cur_node_id])
            self.trajectories.append(trajectories[cur_node_id])
            # TODO: check if real state matches expected state
            # TODO: Flag replan if accidents occurs


    def _publish_node_traj(self, node):
        msg = dict_to_lcmt_manipulator_traj(node)

        if USE_TORQUE:
            msg.dim_torques = 7
        else:
            msg.dim_torques = 0

        if msg.n_time_steps:
            self._lcm.publish("COMMITTED_ROBOT_PLAN", msg.encode())
            print("Trajectory Published!")



#########################   Public Run Function   ##############################
    def save_traj(self):
        """ save trajectory
        """
        if not self.trajectories:
            print("No available trajectory.")
            return

        file_dir = os.getcwd()
        foldername = "/results/"
        try:
            os.stat(file_dir+foldername)
        except:
            os.makedirs(file_dir+foldername)

        filename = "traj"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"

        with open(file_dir+foldername+filename, 'w') as out:
            json.dump(self.trajectories, out)

    def run(self):
        self._pre_plan(self.tree_horizon)

        print("Pre-plan Completed, waiting for simulation to start...")
        while (not self._is_sim_started):
            self._lcm.handle()
        
        while (not self.task_cur.goal_reached(self._executed_state)):
            if len(self.trees) and (not self._executing_tree):
                print("start tree execution process")
                execution_process = Process(
                    target=self._execute_tree, args=(self.trees.pop(0),))
                execution_process.start()

            if (not self._planning) and len(self.unplanned_subproblems) and (
                len(self.trees) < self.tree_horizon):
                print("start planning process")
                sp = self.unplanned_subproblems.pop(0)
                planning_process = Process(
                    target=self._plan_subproblem, args=(sp, self._planned_state))
                planning_process.start()

            if (not self._execute_tree) and (not self._planning) and (
                not len(self.trees)) and (not len(self.unplanned_subproblems)):
                break
        
        self.save_traj()

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
    domain_file = "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/domain_coupled.pddl"
    problem_file = "/home/zhigen/code/pddl_planning/examples/strips/conveyor_belt_multi_grasp_mode/3obj_coupled.pddl"
    geo_setup_file = "/home/zhigen/code/pddl_planning/test/geo_setup.json"
    traj_setup_file = "/home/zhigen/code/pddl_planning/test/traj_setup.json"

    problem = _parse(domain_file, problem_file)
    task = _ground(problem)

    planner = ConveyorBeltManipReactivePlanner(task, geo_setup_file, traj_setup_file)

    planner.run()

if __name__=="__main__":
    main()
