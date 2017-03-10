#!/bin/bash
# arg1 : ROS_MASTER_URI
# arg2 : ROS_IP
echo Enter ROS_MASTER_URI
read varmasteruri
echo Enter ROS_IP
read varrosip

source /opt/ros/kinetic/setup.bash
source ~/roomba_ws/devel/setup.bash
export ROS_MASTER_URI=$varmasteruri
export ROS_IP=$varrosip
roslaunch elikos_roomba rpi_local.launch