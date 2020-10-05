## Conveyor Belt Task and Motion Planning

## Setup
drake setup under a new branch from this branch
```
git clone https://github.gatech.edu/GeorgiaTechLIDARGroup/drake.git
cd drake
git checkout manipulation-tamp-zhigen -b <new-branch-name>
sudo ./setup/ubuntu/install_prereqs.sh
```

pydrake installation
```
cd /path/to/drake
mkdir -p build/install
bazel run //:install -- ~+/build/install
```

add pydrake to PYTHONPATH<br />
Ubuntu 20.04
```
echo "export PYTHONPATH=/path/to/drake/build/install/lib/python3.8/site-packages:${PYTHONPATH} >> ~/.bashrc
source ~/.bashrc
```
Ubuntu 18.04
```
echo "export PYTHONPATH=/path/to/drake/build/install/lib/python3.6/site-packages:${PYTHONPATH} >> ~/.bashrc
source ~/.bashrc
```

## build this project
```
cd /path/to/drake
bazel build //tools:drake_visualizer
bazel build //conveyor_belt_tamp/...
```

## run static object sorting planner
```
cd /path/to/drake
./bazel-bin/conveyor_belt_tamp/pddl_multi_wp_query_handler

cd /path/to/drake/conveyor_belt_tamp/pddl
python3 object_sorting_plan.py
```

## run result from static object sorting planner
result will be saved under //conveyor_belt_tamp/pddl/results folder<br />
open //conveyor_belt_tamp/pddl/json_traj_runner.py<br />
find the result json file you need to run, change JSON_FILENAME variable in json_traj_runner.py<br />

run in this order
```
./bazel-bin/tools/drake_visualizer

./bazel-bin/conveyor_belt_tamp/kuka_planner_runner

cd /path/to/drake/conveyor_belt_tamp/pddl/
python3 json_traj_runner.py

./bazel-bin/conveyor_belt_tamp/object_sorting_simulation
```