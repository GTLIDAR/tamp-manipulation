import numpy
import json

filename = "/home/zhigen/code/drake/traj_gen/trajectory_data/fixvec_0-1-2-3-4-5-6-7-8-9-10-11-12-13.json"
with open(filename) as fp:
    data = json.load(fp)

print("q final:")
print(data[-1]["states"][-1])

