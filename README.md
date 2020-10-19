## Manipulation Task and Motion Planning
This repository contains code for manipulation task and motion planning using PDDL and DDP/ADMM.

## Drake

The code is based on Drake (Please see the [Drake Documentation](https://drake.mit.edu) for more
information). Here we include the source code of Drake and our own addition for manipulation task and motion planning in the manipulation_tamp and traj_gen folder.

The code is run on Ubuntu 18.04 and 20.04 and Python 3.6 or newer.

## Setup
drake setup
```
git clone https://github.com/GTLIDAR/tamp-manipulation.git
cd tamp-manipulation
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
echo "export PYTHONPATH=/path/to/drake/build/install/lib/python3.8/site-packages:${PYTHONPATH}" >> ~/.bashrc
source ~/.bashrc
```
Ubuntu 18.04
```
echo "export PYTHONPATH=/path/to/drake/build/install/lib/python3.6/site-packages:${PYTHONPATH}" >> ~/.bashrc
source ~/.bashrc
```

## Setup PDDL planning code and Pyperplan submodule
```
sudo apt install python3 python3-pip
python3 -m pip install numpy scipy matplotlib networkx
git submodule update --init --recursive
cd /path/to/drake/manipulation_tamp/planner/pyperplan
sudo python3 setup.py install
```


## build this project
Ubuntu 18.04
```
cd /path/to/drake
bazel build //tools:drake_visualizer
bazel build //manipulation_tamp/...
```

Ubuntu 20.04
```
sudo apt install clang
cd /path/to/drake
CC=clang CXX=clang++ bazel build //tools:drake_visualizer
CC=clang CXX=clang++ bazel build //manipulation_tamp/...
```

## run static object sorting planner
```
cd /path/to/drake
./bazel-bin/manipulation_tamp/pddl_multi_wp_query_handler_contact

cd /path/to/drake/manipulation_tamp/planner
python3 object_sorting_plan.py
```

## run static object sorting planner
```
cd /path/to/drake
./bazel-bin/manipulation_tamp/pddl_multi_wp_query_handler_contact

cd /path/to/drake/manipulation_tamp/planner
python3 object_sorting_planner.py
```

## run result from static object sorting planner
result will be saved under //manipulation_tamp/results folder<br />
open //manipulation_tamp/json_traj_runner.py<br />
find the result json file you need to run, change JSON_FILENAME variable in json_traj_runner.py<br />

run in this order
```
./bazel-bin/tools/drake_visualizer

./bazel-bin/manipulation_tamp/kuka_planner_runner

cd /path/to/drake/manipulation_tamp
python3 json_traj_runner.py

./bazel-bin/manipulation_tamp/object_sorting_simulation
```

## run static conveyor belt planner
```
cd /path/to/drake
./bazel-bin/manipulation_tamp/pddl_multi_wp_query_handler_contact

cd /path/to/drake/manipulation_tamp/planner
python3 conveyor_belt_planner.py
```

## run result from static object sorting planner
result will be saved under //manipulation_tamp/results folder<br />
open //manipulation_tamp/json_traj_runner.py<br />
find the result json file you need to run, change JSON_FILENAME variable in json_traj_runner.py<br />

run in this order
```
./bazel-bin/tools/drake_visualizer

./bazel-bin/manipulation_tamp/kuka_planner_runner

cd /path/to/drake/manipulation_tamp
python3 json_traj_runner.py

./bazel-bin/manipulation_tamp/conveyor_belt_simulation
```