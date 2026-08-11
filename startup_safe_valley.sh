#!/bin/bash

# 将 ROS 运行日志重定向到当前工作空间 .ros_home
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"

# >>> fishros initialize >>>
source /opt/ros/noetic/setup.bash
# <<< fishros initialize <<<
cd ~
source ~/catkin_ws/devel/setup.bash
sleep 2

roslaunch safe_valley_exp uav_offboard_real.launch