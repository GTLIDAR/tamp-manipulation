import time
import os
import sys
from datetime import datetime
import copy
import json
import numpy as np

from lcm import LCM
from drake import lcmt_combined_object_state, lcmt_iiwa_status, lcmt_schunk_wsg_status

pddl_path = "/home/zhigen/code/pddl_planning"
sys.path.append(pddl_path)

from causal_graph.tools import build_causal_graph, get_subproblems, generate_subtask
from search_tree.tamp_node import PddlTampNode
from search_tree.tamp_tree import PddlTree

from pyperplan import _parse, _ground

class ConveyorBeltManipReactivePlanner:
    """ A planner based on causal graph subtask separation

    Attributes:
        task_orig: (pyperplan.Task) original task
        task_cur: (pyperplan.Task) current remaining task
        causal_graph: (networkx DiGraph) Original causal graph generated from task
        subproblems: a list with (subgoals, ops) pair where "subgoals" is a set
            of subgoals in a disconnected component, and "ops" is a set of operators
            related to the component
        trajectories: (list of nodes) trajectories
        actions: (list of pyperplan.Operation)
        trees: (list of PDDLTrees)
    """
    def __init__(self, task, geo_setup_file, traj_setup_file):
        self._object_state = None
        self._iiwa_status = None
        self._wsg_status = None

        self._lcm = LCM()
        self._lcm.subscribe("OBJECT_STATE", self._object_state_handler)
        self._lcm.subscribe("IIWA_STATUS", self._iiwa_status_handler)
        self._lcm.subscribe("SCHUNK_WSG_STATUS", self._wsg_status_handler)

        with open(geo_setup_file, 'r') as fp:
            self._geo_setup = json.load(fp)

        with open(traj_setup_file, 'r') as fp:
            self._traj_setup = json.load(fp)

        self.task_orig = task
        self.task_cur = task
        self.causal_graph = build_causal_graph(task, show=False)
        self.subproblems = []
        self.trajectories = []
        self.actions = []
        self.trees = []

    def _object_state_handler(self, channel, msg):
        self._object_state = lcmt_combined_object_state.decode(msg)

    def _iiwa_status_handler(self, channel, msg):
        self._iiwa_status = lcmt_iiwa_status.decode(msg)

    def _wsg_status_handler(self, channel, msg):
        self._wsg_status = lcmt_schunk_wsg_status.decode(msg)

    def _update_remaining_task(self):
        pass

    def run_plan(self):
        pass

def main():
    pass

if __name__=="__main__":
    main()
