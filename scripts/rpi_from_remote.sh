#!/bin/bash
# SSH into RPi and run:
#   start_roscore.sh
#   rpi_local.sh
# arg1 : username
# arg2 : ip address
# arg3 : password
screen -d -m -S pikos bash -c '. ~/roomba_ws/src/elikos_roomba/scripts/start_roscore.sh'&
screen -d -m -S pikos bash -c '. ~/roomba_ws/src/elikos_roomba/scripts/rpi_local.sh'&