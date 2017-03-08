# elikos_roomba
Custom software for ground &amp; osbtacle robots with on-board Raspberry Pi

**Status**: WIP


## How to

1. Fork this repo and work on that (clone your own fork)

   1. Set `upstream` as  
      ````
      git remote add upstream git@github.com:elikos/elikos_roomba.git
      ````  

2. Create a pull request


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

2. Fork the [`elikos/elikos_roomba`](https://github.com/elikos/elikos_roomba) repo  
   Clone your own fork in the `~/roomba_ws/src` folder
   ````
   git clone git@github.com:USERNAME/elikos_roomba.git
   ````  
   with `USERNAME` being your GitHub username

3. Clone the [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package repo in the `~/roomba_ws/src` folder
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

   3. Serial error, with a message like  
      ````
      [create::Serial] failed to receive data from Create. Check if robot is powered!
      ````  
      and no serial communication *from* the robot:  
         1. no blue light on the USB connector of the USB-to-serial cable; or  
         2. no response when doing `cat /dev/ttyUSB0 115200`

      Remove the 4 screws and then remove the battery. Wait a couple seconds, then put it back in along with the screws. You should hear a little happy tune and serial communication should now work.


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
