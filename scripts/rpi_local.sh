#!/bin/bash
# launch locally
echo Enter ROS_MASTER_URI (http://192.168.x.y:11311)
read varmasteruri
echo Enter ROS_IP (192.168.x.y)
read varrosip

source /opt/ros/kinetic/setup.bash
source ~/roomba_ws/devel/setup.bash
export ROS_MASTER_URI=$varmasteruri
export ROS_IP=$varrosip
roslaunch elikos_roomba rpi_local.launch