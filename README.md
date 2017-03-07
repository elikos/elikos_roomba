# elikos_roomba
Custom software for ground &amp; osbtacle robots with on-board Raspberry Pi

## Prerequisites

1. Raspberry Pi with [Ubuntu MATE 16.04](https://ubuntu-mate.org/raspberry-pi/) and [ROS kinetic (Ubuntu/armhf)](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. Install [WiringPi (library)](http://wiringpi.com/download-and-install/)
   ````
   sudo apt-get purge wiringpi
   hash -r
   cd
   git clone git://git.drogon.net/wiringPi
   cd ~/wiringPi
   ./build
   ````

## Initial setup

1. Create workspace directory and init the catkin workspace
   ````
   mkdir -p ~/roomba_ws/src
   cd ~/roomba_ws/src
   catkin_init_workspace
   ````

2. Clone repo in the `~/roomba_ws/src` folder
   ````
   git clone git@github.com:christophebedard/elikos_roomba.git
   ````

3. Clone [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package repo in the `~/roomba_ws/src` folder
   ````
   git clone git@github.com:AutonomyLab/create_autonomy.git
   sudo usermod -a -G dialout $USER
   ````

4. Compile
   ````
   cd ~/roomba_ws
   catkin_make
   ````

5. Source 
   ````
   . ./devel/setup.bash
   ````

6. Do stuff