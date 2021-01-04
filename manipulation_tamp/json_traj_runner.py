#!/usr/bin/env python3

import json
import os

from lcm import LCM
from drake import lcmt_schunk_wsg_command, lcmt_schunk_wsg_status, lcmt_manipulator_traj, lcmt_generic_string_msg

from planner.utils.traj_utils import dict_to_lcmt_manipulator_traj

file_path = os.path.dirname(os.path.abspath(__file__))
drake_path = file_path+"/.."
result_path = drake_path+"/manipulation_tamp/results/"

# JSON_FILENAME = result_path + "fixed_admm_traj20201023T000819.json"
JSON_FILENAME = "/home/zhigen/code/drake/manipulation_tamp/results/admm_traj20210104T100635.json"
# JSON_FILENAME = "/home/zhigen/code/drake/manipulation_tamp/results/22_updated_full.json"

REMOVE_LAST_STEP = False
USE_TORQUE = False

class JsonManipulatorTrajRunner:
    def __init__(self, filename):
        with open(filename) as fp:
            self.plan = json.load(fp)

        self._lcm = LCM()
        self._lcm.subscribe("EXECUTION_STATUS", self._kuka_plan_runner_status_handler)

        self.wsg_utime = 0
        self.torque_mode = 0
        self.cur_node_id = 0
        self.plan_started = False
        self.node_completed = False

    def run_plan(self):
        if len(self.plan):
            print("Node", self.cur_node_id)
            self._publish_node_traj(self.plan[self.cur_node_id])

            while True:
                self._lcm.handle()
                if self.node_completed:
                    self.node_completed = False
                    self.cur_node_id += 1
                    if self.cur_node_id >= len(self.plan):
                        print("Plan Completed")
                        break
                    print("Node", self.cur_node_id)
                    self._publish_node_traj(self.plan[self.cur_node_id])


    def _publish_node_traj(self, node):
        msg = dict_to_lcmt_manipulator_traj(node)

        if USE_TORQUE:
            msg.dim_torques = 7
        else:
            msg.dim_torques = 0

        if msg.n_time_steps:
            self._lcm.publish("COMMITTED_ROBOT_PLAN", msg.encode())
            print("Trajectory Published!")

    def _kuka_plan_runner_status_handler(self, channel, msg):
        print("Kuka Traj Completed")
        self.node_completed = True

    def run_plan_lcm_wrapper(self):
        self._lcm.subscribe("START_PLAN", self._start_plan_handler)
        while (not self.plan_started):
            self._lcm.handle()

    def _start_plan_handler(self, channel, msg):
        self.plan_started = True
        print("Starting plan")
        self.run_plan()

def main():
    runner = JsonManipulatorTrajRunner(JSON_FILENAME)
    runner.run_plan_lcm_wrapper()

if __name__=="__main__":
    main()
