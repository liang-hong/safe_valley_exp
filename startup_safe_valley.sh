#!/bin/bash

# >>> fishros initialize >>>
source /opt/ros/noetic/setup.bash
# <<< fishros initialize <<<
cd ~
source ~/catkin_ws/devel/setup.bash
sleep 2

roslaunch safe_valley_exp uav_offboard_real.launch