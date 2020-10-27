import json
import os

file_path = os.path.dirname(os.path.abspath(__file__))
drake_path = file_path+"/.."
result_path = drake_path+"/manipulation_tamp/results/"

JSON_FILENAME = "admm_traj20201023T000819.json"
JSON_FILENAME_2 = "fixed_admm_traj20201023T000819.json"

with open(result_path+JSON_FILENAME, 'r') as fp:
    data = json.load(fp)

for node in data:
    add_time = 0
    ts = node["times_sec"][1] - node["times_sec"][0]
    for i in range(node["n_time_steps"]-1):
        if node["times_sec"][i] >= node["times_sec"][i+1]:
            node["times_sec"][i] += add_time
            add_time += ts
            print("added time", add_time)
        else:
            node["times_sec"][i] += add_time

        if i == node["n_time_steps"]-2:
            node["times_sec"][i+1] += add_time

with open(result_path+JSON_FILENAME_2, 'w') as out:
    json.dump(data, out)

