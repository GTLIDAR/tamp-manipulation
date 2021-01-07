import json
import os

file_path = os.path.dirname(os.path.abspath(__file__))
drake_path = file_path+"/../.."

JSON_FILENAME = drake_path + "/manipulation_tamp/results/con27.json"

TIMING_FACTOR = 0.79

with open(JSON_FILENAME) as fp:
    data = json.load(fp)

n_ts = len(data[-1]["gripper_width"])
for i in range(n_ts):
    if i < n_ts * TIMING_FACTOR:
        data[-1]["gripper_width"][i] = 10
        print(i)
    else:
        data[-1]["gripper_width"][i] = 100

with open(JSON_FILENAME, 'w') as out:
    json.dump(data, out)