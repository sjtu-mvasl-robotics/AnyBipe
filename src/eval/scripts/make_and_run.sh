#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "DIR: $DIR"
cd $DIR/../../../biped_ws
catkin_make install
source install/setup.bash
exec roslaunch robot_hw pointfoot_hw_sim_headless.launch
