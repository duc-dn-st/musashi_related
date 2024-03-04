#!/bin/bash

echo "Create map at current time"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

MY_PATH="${SCRIPT_DIR}/src/cartographer_config/maps/map.pbstream"

source ../cartographer_ws/devel_isolated/setup.bash
# rosrun map_server map_saver -f $(rospack find "lumi_robot")/maps/mymap
# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '$MY_PATH', include_unfinished_submaps: "true"}"

