# MPCC

## Installation 
To install all the dependencies run
```
./install.sh
```

## How to run
rosrun model_predictive_contouring_control MPCC

## for running on SIMULATION
change main2.cpp -> main.cpp
when use for experiment vise versa.

## Parameters
Parameter settings can be found in 'Param' folder.
- bounds.json for constraints
- cost.json for penalty weights
- model.json for model size settings
- config.json for sampling time etc

## Check directories in following files if error
- CMakeLists.txt (line 73)
- config.json (line 9-13)