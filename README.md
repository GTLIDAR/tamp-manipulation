## Manipulation Task and Motion Planning
This repository contains code for our recent work on symbolic-decision-embedded bilevel optimization for manipulation task and motion planning using PDDL and DDP/ADMM.

The code is based on Drake (Please see the [Drake Documentation](https://drake.mit.edu) for moreinformation). Here we include the source code of Drake and our own addition for manipulation task and motion planning in the [manipulation_tamp](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/manipulation_tamp) and [traj_gen](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/traj_gen) folder.

The [manipulation_tamp](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/manipulation_tamp) directory contains implementations of the [high level searching algorithm](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/manipulation_tamp/planner/search_tree) as well as the [causal graph task decomposition](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/manipulation_tamp/planner/causal_graph). The symbolic planning domain definitions of our experiments are under the [pddl](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/manipulation_tamp/pddl) directory. The planner uses [Pyperplan](https://github.com/zhigenzhao/pyperplan/tree/7339885e913ad40ba2be020a7b7940619b6da67b) as a submodule to parse PDDL into Python. The resulting trajectories and action sequences are stored in Json format under manipulation_tamp/results folder. The simulation of object sorting in clutter and conveyor belt sorting scenarios are implemented here using Drake.

The [traj_gen](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/traj_gen) directory contains the code for trajectory optimization using DDP/ADMM. An example of how to use DDP/ADMM planners will be released soon. The contact version of DDP/ADMM that considers object dynamics is also implemented under [ilqr_kkt](https://github.com/GTLIDAR/tamp-manipulation/tree/manipulation-tamp-RAL/traj_gen/ilqr_kkt) directory. The runners ending in the "-contact" suffix will be called when selecting high-level actions that involve contacts between the gripper and the object, such as grasping or pushing. 

## Setup
The code is tested on Ubuntu 18.04 and 20.04 and Python 3.6 or newer.

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

## Project and Related Publications
This work is a part of our ongoing work on task and motion planning for manipulation in dynamic and cluttered environments.

This repo contains the code used for implementation in our work [SyDeBO: Symbolic-Decision-Embedded Bilevel Optimization for Long-Horizon Manipulation in Dynamic Environments](https://arxiv.org/abs/2010.11078).

```
@misc{zhao2020sydebo,
      title={SyDeBO: Symbolic-Decision-Embedded Bilevel Optimization for Long-Horizon Manipulation in Dynamic Environments}, 
      author={Zhigen Zhao and Ziyi Zhou and Michael Park and Ye Zhao},
      year={2020},
      eprint={2010.11078},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
