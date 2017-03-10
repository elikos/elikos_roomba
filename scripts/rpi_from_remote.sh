#!/bin/bash
# SSH into RPi and run:
#   start_roscore.sh
#   rpi_local.sh
# arg1 : username
# arg2 : ip address
# arg3 : ROS_MASTER_URI
# arg4 : ROS_IP
ssh $1@$2
#screen -d -m -S pikos bash -c '. ~/roomba_ws/src/elikos_roomba/scripts/start_roscore.sh &. ~/roomba_ws/src/elikos_roomba/scripts/rpi_local.sh'
. ~/roomba_ws/src/elikos_roomba/scripts/start_roscore.sh $3 $4 & . ~/roomba_ws/src/elikos_roomba/scripts/rpi_local.sh $3 $4