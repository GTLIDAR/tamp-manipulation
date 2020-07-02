import json
from builtins import input
import sys
import os
import copy
import time

from lcm import LCM
from drake import lcmt_schunk_wsg_command, lcmt_schunk_wsg_status, lcmt_ddp_traj, lcmt_generic_string_msg


file_dir = os.path.dirname(os.path.abspath(__file__))

GRIPPER_WIDTH = [100, 20]
GRIPPER_FORCE = 80

JSON_FILE_NAME = "/home/zhigen/code/lidar-drake/conveyor_belt_iiwa/pddl/results/20200521T054047/traj0.json"
REMOVE_LAST_STEP = True

class JsonResultsRunner():
    def __init__(self, filename):
        with open(filename) as fp:
            self.plan = json.load(fp)

        self.lcm_ = LCM()
        self.lcm_.subscribe("EXECUTION_STATUS", self.kuka_plan_runner_status_handler)
        # self.lcm_.subscribe("SCHUNK_WSG_STATUS", self.wsg_status_handler)
        self.wsg_utime = 0
        self.torque_mode = 0
        self.cur_node_id = 0
        # self.plan_completed = False
        self.plan_started = False
        self.node_completed = False

        # for back compatibility
        self.node_start_time = 0

    def run_plan(self):
        if len(self.plan):
            print("Node", self.cur_node_id, self.plan[self.cur_node_id]["name"])
            self.publish_node_traj(self.plan[self.cur_node_id])

            while True:
                self.lcm_.handle()
                if self.node_completed:
                    self.node_completed = False
                    self.cur_node_id += 1
                    if self.cur_node_id >= len(self.plan):
                        print("Plan Completed")
                        break
                    print("Node", self.cur_node_id, self.plan[self.cur_node_id]["name"])
                    self.publish_node_traj(self.plan[self.cur_node_id])
        
        else:
            print("No available plan")

    
    def get_lcmt_ddp_traj(self, node_data):
        msg = lcmt_ddp_traj()
        if not REMOVE_LAST_STEP:
            msg.n_time_steps = len(node_data["times_sec"])
            
            msg.dim_states = 7
            msg.states = node_data["q"]
            msg.torques = node_data["u"]
            msg.times_sec = node_data["times_sec"]

            msg.gripper_width = node_data["gripper_width"]
            msg.gripper_force = node_data["gripper_force"]
        else:
            msg.n_time_steps = len(node_data["times_sec"]) - 1
            
            msg.dim_states = 7
            msg.states = node_data["q"][:-1]
            msg.torques = node_data["u"][:-1]
            msg.times_sec = node_data["times_sec"][:-1]

            msg.gripper_width = node_data["gripper_width"][:-1]
            msg.gripper_force = node_data["gripper_force"][:-1]
        
        if self.torque_mode:
            msg.dim_torques = 7
        else:
            msg.dim_torques = 0

        return msg

    def publish_node_traj(self, node_data):
        msg = self.get_lcmt_ddp_traj(node_data)
        if msg.n_time_steps:
            self.lcm_.publish("COMMITTED_ROBOT_PLAN", msg.encode())
            print("Trajectory Published!")

    def kuka_plan_runner_status_handler(self, channel, msg):
        print("Kuka Traj Completed")
        self.node_completed = True
    
    def run_plan_lcm_wrapper(self):
        self.lcm_.subscribe("START_PLAN", self.start_plan_handler)
        while (not self.plan_started):
            self.lcm_.handle()
        
    def start_plan_handler(self, channel, msg):
        self.plan_started = True
        print("Starting plan")
        self.run_plan()    

def main():
    runner = JsonResultsRunner(JSON_FILE_NAME)
    runner.run_plan_lcm_wrapper()
    
if __name__=="__main__":
    main()
