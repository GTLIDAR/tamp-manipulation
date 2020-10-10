import json
import math
import random
from datetime import datetime
import time

from lcm import LCM
from drake import lcmt_multi_wp_manip_query, lcmt_manipulator_traj

from utils.traj_utils import dict_to_lcmt_multi_wp_manip_query, lcmt_manipulator_traj_to_dict

JSON_QUERY_FILENAME = "/home/zhigen/code/drake/conveyor_belt_tamp/results/admm_queries20201010T155958.json"
JSON_DDP_TRAJ_FILENAME = "/home/zhigen/code/drake/conveyor_belt_tamp/results/traj20201010T155957.json"

MIN_TIME_STEP = 0.001

class MotionQueryHandler:
    def __init__(self, query_file, ddp_traj_file):
        with open(query_file) as fp:
            data = json.load(fp)
        
        self._queries = []
        for d in data:
            try:
                self._queries.append(dict_to_lcmt_multi_wp_manip_query(d))
            except:
                self._queries.append(None)

        with open(ddp_traj_file) as fp:
            self._ddp_traj = json.load(fp)
        
        self._lcm = LCM()
        self._lcm.subscribe("TREE_SEARCH_QUERY_RESULTS", self._query_results_handler)

        self._cur_query = None
        self._completed = False
        self.trajs = []
    
    def _query_results_handler(self, channel, msg):
        print("message received.")
        data = lcmt_manipulator_traj.decode(msg)
        print("Cost:", data.cost)

        if not data.n_time_steps:
            print("IK Infeasible")
        elif data.cost==float('inf') or math.isnan(data.cost) or data.cost==0:
            if self._cur_query.time_step/2 >= MIN_TIME_STEP:
                self._cur_query.time_step /= 2
                self._lcm.publish("TREE_SEARCH_QUERY", self._cur_query.encode())
                print("Decreasing time step to",
                    self._cur_query.time_step)
                return
            else:
                print("Minimum time step reached. Trajectory cannot be found.")
        
        self.trajs.append(lcmt_manipulator_traj_to_dict(data))
        self._completed = True

    def Run(self):
        q_idx = 0
        total_time = 0
        for q_idx in range(len(self._queries)):
            self._cur_query = self._queries[q_idx]
            if self._cur_query is None:
                print("No move query in this action. Keeping original trajectory")
                self.trajs.append(self._ddp_traj[q_idx])
                continue

            print("Computing ADMM "+str(q_idx+1)+"/"+str(len(self._queries)))
            print("Action: "+self._cur_query.name)

            self._completed = False
            start_time = time.time()
            self._lcm.publish("TREE_SEARCH_QUERY", self._cur_query.encode())
            print("LCM Query Published...")

            while not self._completed:
                self._lcm.handle()
            
            end_time = time.time()
            total_time += end_time-start_time
            print("Computation time:", end_time-start_time)
        
        print("Total computation time:", total_time)
        self._save_trajs()
    
    def run_single_traj(self, q_idx):
        self._cur_query = self._queries[q_idx]
        
        if self._cur_query is None:
            print("No move query in this action. Keeping original trajectory")
            self.trajs.append(self._ddp_traj[q_idx])
        
        else:
            print("Computing ADMM "+str(q_idx+1)+"/"+str(len(self._queries)))
            print("Action: "+self._cur_query.name)

            self._completed = False
            start_time = time.time()
            self._lcm.publish("TREE_SEARCH_QUERY", self._cur_query.encode())
            print("LCM Query Published...")

            while not self._completed:
                self._lcm.handle()
        
        self._save_trajs()

    
    def _save_trajs(self):
        print("ADMM completed, saving results")

        filename = "/home/zhigen/code/drake/conveyor_belt_tamp/results/admm_traj"+datetime.now().strftime("%Y%m%dT%H%M%S")+".json"
        
        with open(filename, 'w') as out:
            json.dump(self.trajs, out)
            
def main():
    runner = MotionQueryHandler(JSON_QUERY_FILENAME, JSON_DDP_TRAJ_FILENAME)
    runner.Run()
    # runner.run_single_traj(6)

if __name__ == "__main__":
    main()