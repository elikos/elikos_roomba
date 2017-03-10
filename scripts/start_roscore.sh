#!/bin/bash
# arg1 : ROS_MASTER_URI
# arg2 : ROS_IP
source /opt/ros/kinetic/setup.bash
source ~/roomba_ws/devel/setup.bash
export ROS_MASTER_URI=$1
export ROS_IP=$2
roscore