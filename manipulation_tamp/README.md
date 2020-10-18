## Manipulation Task and Motion Planning

## Setup
drake setup
```
git clone https://github.gatech.edu/GeorgiaTechLIDARGroup/drake.git
cd drake
git checkout manipulation-tamp-RAL
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

## setup pddl planning code and pyperplan submodule
```
sudo apt install python3 python3-pip
python3 -m pip install numpy scipy matplotlib networkx
git clone https://github.gatech.edu/GeorgiaTechLIDARGroup/pddl_planning.git
cd pddl_planning
git submodule update --init --recursive
cd /path/to/drake/manipulation_tamp/planner/pyperplan
sudo ./setup.py install
source ~/.bashrc
```


## build this project
```
cd /path/to/drake
bazel build //tools:drake_visualizer
bazel build //manipulation_tamp/...
```

## run static object sorting planner
```
cd /path/to/drake
./bazel-bin/manipulation_tamp/pddl_multi_wp_query_handler_contact

cd /path/to/drake/manipulation_tamp/planner
python3 object_sorting_plan.py
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