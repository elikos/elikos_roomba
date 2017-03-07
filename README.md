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


## Troubleshooting

Mostly because Christophe is dumb and often forgets.

* Error on `roslaunch` of either `ca_driver` or `ca_tools joy_teleop`

   1. Make sure your your user is in the dialout group (couldn't access USB peripherals otherwise)  
      ````
      sudo usermod -a -G dialout $USER
      ````  
      then logout and login again

   2. Make sure [`joy`](http://wiki.ros.org/joy) is installed  
      ````
      sudo apt-get install ros-kinetic-joy
      ````

* Can't run ROS or `roslaunch` `ca_tools joy_teleop` from another computer  
   See [ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

   1. Configure `ROS_MASTER_URI` for every terminal on the remote machine  
      ````
      export ROS_MASTER_URI=http://192.168.x.y:11311
      ````  
      with `192.168.x.y` being the actual address of the host machine (Raspberry Pi); check with `hostname -I`

* No communication between the host and remote machines  
   Symptom: `Couldn't find an AF_INET address for [HOSTNAME]` message on the `roscore` of the host

   1. Configure `ROS_IP` for (every terminal?) on the remote computer  
      ````
      export ROS_IP=192.168.x.y
      ````  
      find the `192.168.x.y` address with `hostname -I`
