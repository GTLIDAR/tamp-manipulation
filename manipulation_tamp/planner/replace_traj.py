import json
import os

file_path = os.path.dirname(os.path.abspath(__file__))
drake_path = file_path+"/../.."

JSON_DEST_FILENAME = drake_path + "/manipulation_tamp/results/admm_traj20210104T084414.json"
JSON_SRC_FILENAME = drake_path + "/manipulation_tamp/results/22_updated.json"
JSON_OUT_FILENAME = drake_path + "/manipulation_tamp/results/22_updated_full.json"

with open(JSON_DEST_FILENAME) as fp:
    dest = json.load(fp)

with open(JSON_SRC_FILENAME) as fp:
    src = json.load(fp)

dest[22] = src[0]

with open(JSON_OUT_FILENAME, 'w') as out:
    json.dump(dest, out)

