"""
1) read generated json file and plot all seven joints
2) separate file 
- subscribe to robot message and save as json (message name should be the 
same for the simulation and the real hardware)
3) read three different json files for joint trajectories and plot
"""

import json
import sys
import os
import copy
import numpy as np
import matplotlib as plt
from datetime import datetime

from lcm import LCM
from drake import lcmt_iiwa_status, lcmt_manipulator_traj

sys.path.append("/home/zhigen/code/drake/manipulation_tamp")
from planner.utils.traj_utils import lcmt_manipulator_traj_to_dict

# JSON_FILENAME = "/home/zzhao300/code/drake/conveyor_belt_tamp/results/traj20201004T025330.json"
# REMOVE_LAST_STEP = False
# USE_TORQUE = False

# with open(JSON_FILENAME) as fp:
#     plan = json.load(fp)

# # plan[0]["times_sec"] ->plan[first motion][field][timestep]
# # plan[0]["states"][0][0~6] -> joints
# time = plan[0]["times_sec"]
# joint = plan[0]["states"]

# j0, j1, j2, j3, j4, j5, j6 = ([] for i in range(7))
# jdict = {0:j0, 1:j1, 2:j2, 3:j3, 4:j4, 5:j5, 6:j6}

# for t, j in zip(time,joint):
#     for i, nj in enumerate(j):
#         print(i, nj)
#         jdict[i].append(nj)

# print (jdict)

LOG_TIME = 5

class JointTrajecoryPlotter:
    def __init__(self):
        self._iiwa_status = None
        self._traj = dict()
        self._traj = dict()
        self._traj["n_time_steps"] = 0
        self._traj["dim_torques"] = 7
        self._traj["dim_states"] = 7
        self._traj["times_sec"] = []
        self._traj["gripper_width"] = []
        self._traj["gripper_force"] = 0
        self._traj["joint_torque_commanded"] = []
        self._traj["joint_torque_measured"] = []
        self._traj["joint_position_commanded"] = []
        self._traj["joint_position_measured"] = []
        self._traj["joint_velocity_estimated"] = []

        self._lcm = LCM()
        # self._lcm.subscribe("IIWA_STATUS", self._iiwa_status_handler)

        self._start_time = -1
        self._cur_time = 0

        self.plan_started = False

    def _iiwa_status_handler(self, channel, msg):

        self._iiwa_status = lcmt_iiwa_status.decode(msg)
        
        if self._start_time == -1:
            print("Received first status")
            self._start_time = self._iiwa_status.utime/1e6
        
        self._cur_time = self._iiwa_status.utime/1e6-self._start_time
        self._traj["times_sec"].append(self._iiwa_status.utime/1e6-self._start_time)
        self._traj["n_time_steps"] += 1
        self._traj["joint_position_commanded"].append(self._iiwa_status.joint_position_commanded)
        self._traj["joint_position_measured"].append(self._iiwa_status.joint_position_measured)
        self._traj["joint_torque_commanded"].append(self._iiwa_status.joint_torque_commanded)
        self._traj["joint_torque_measured"].append(self._iiwa_status.joint_torque_measured)
        self._traj["joint_velocity_estimated"].append(self._iiwa_status.joint_velocity_estimated)
    
    def _save_json(self):
        print("saving json file")
        foldername = "/home/zhigen/code/drake/traj_gen/trajectory_data/"
        filename = "iiwa_status"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"
        with open(foldername+filename, 'w') as out:
            json.dump(self._traj, out)

    def Run(self):
        self._lcm.subscribe("IIWA_STATUS", self._iiwa_status_handler)
        while True:
            self._lcm.handle()
            if self._cur_time >= LOG_TIME:
                self._save_json()
                break

    def run_plan_lcm_wrapper(self):
        self._lcm.subscribe("COMMITTED_ROBOT_PLAN", self._start_plan_handler)
        while (not self.plan_started):
            self._lcm.handle()

    def _start_plan_handler(self, channel, msg):
        if not self.plan_started:
            self.plan_started = True
            print("Starting plan")
            self.Run()
            self._save_commanded_traj(msg)
    
    def _save_commanded_traj(self, msg):
        data = lcmt_manipulator_traj_to_dict(
            lcmt_manipulator_traj.decode(msg)
        )
        print("saving commanded trajectory")
        foldername = "/home/zhigen/code/drake/traj_gen/trajectory_data/"
        filename = "iiwa_commanded_traj"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"
        with open(foldername+filename, 'w') as out:
            json.dump([data], out)



def main():
    plotter = JointTrajecoryPlotter()
    #plotter.Run()
    plotter.run_plan_lcm_wrapper()

if __name__ == "__main__":
    main()


            


