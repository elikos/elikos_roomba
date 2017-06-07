# elikos_roomba
ROS package for ground &amp; osbtacle robots with on-board Raspberry Pi

**Status**: working ground robot and obstacle robot



`elikos_roomba` works with the [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package to communicate with and drive the Roomba with ROS. Its purpose is to implement the IARC mission 7 ground robot and obstacle [behaviours](#robot-behaviour).

It interacts with `create_autonomy` through its topics. It also has a node to manage the ground robot's top switch and call a service when activated.

| `elikos_roomba`     |  Topic/service        | Description                                                                                             |
| :------------------ | :-------------------- |:------------------------------------------------------------------------------------------------------- |
|  subscriber         | `/bumper`             | Bumper state message (published by `create_autonomy`)                                                   |
|  publisher          | `/cmd_vel`            | Drives the robot's wheels according to a forward and angular velocity (`create_autonomy` is subscribed) |
|  service server     | `/topswitch_trigger`  | Top switch trigger call (`topswitch_node` is the service client) |

-----


## Launch files

* `rpi_local.launch`  
   * launches `create_autonomy/ca_driver` with the `topswitch_node`  
   * to be launched locally on the Raspberry Pi

* `robot_ground.launch`  
   * launches `groundrobot_node`  
   * to be launched on a remote computer or on the Raspberry Pi itself

* `robot_obstacle.launch`  
   * launches `obstaclerobot_node`  
   * to be launched on a remote computer or on the Raspberry Pi itself

* `joy_teleop.launch`  
   * launches a `joy_teleop` node  
   * to be launched on a computer with an Xbox 360/Xbox One controller

## Nodes

* `topswitch_node`  
   * manages the top switch on the Raspberry Pi through GPIO

* `groundrobot_node`  
   * ground robot behaviour

* `obstaclerobot_node`  
   * obstacle robot behaviour

## Services

* `/robot_activate_toggle`  
   * activate/deactivate current robot

* `/topswitch_trigger`  
   * usually called by `topswitch_node` when top switch is triggered


-----


## Prerequisites

1. Raspberry Pi with [Ubuntu MATE 16.04](https://ubuntu-mate.org/raspberry-pi/)


2. Install [ROS kinetic (Ubuntu/armhf)](http://wiki.ros.org/kinetic/Installation/Ubuntu) (*-desktop*)
   ````
   sudo apt-get update
   sudo apt-get update
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
   sudo apt-get update
   sudo apt-get install ros-kinetic-desktop
   ````

3. Install [WiringPi (library)](http://wiringpi.com/download-and-install/)
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
   cd elikos_roomba/
   git remote add upstream git@github.com:elikos/elikos_roomba.git
   ````  
   with `USERNAME` being your GitHub username

3. Clone the [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package repo in the `~/roomba_ws/src` folder
   ````
   git clone git@github.com:AutonomyLab/create_autonomy.git
   sudo usermod -a -G dialout $USER
   ````  
   Log out and log back in.

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

7. Create a pull request


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
      
      Try:  
      1. Press the power button once. It should immediately connect.  
      2. If that still doesn't work, force the roomba to reset. Remove the 4 screws and then remove the battery. Wait a couple seconds, then put it back in along with the screws. You should hear a little happy tune and serial communication should now work.


* Can't run ROS or `roslaunch` `ca_tools joy_teleop` from another computer  
   See [ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

   1. Configure `ROS_MASTER_URI` variable pointing to the `roscore` host for every terminal on both the host and remote computers  
      ````
      export ROS_MASTER_URI=http://192.168.x.y:11311
      ````  
      with `192.168.x.y` being the actual address of the host computer; check with `hostname -I`

* No communication between the host and remote computer  
   Symptom: `Couldn't find an AF_INET address for [HOSTNAME]` message on the `roscore` of the host

   1. Configure `ROS_IP` variable pointing to the local computer for every terminal on both the host and remote computers  
      ````
      export ROS_IP=192.168.x.y
      ````  
      find the `192.168.x.y` address with `hostname -I`

## Robot behaviour

Behaviour description of the ground robot and obstacle robot according to the [official rules](http://www.aerialroboticscompetition.org/rules.php).

| Info                 | Ground robot                                  | Obstacle robot                                       |
| :------------------- |:--------------------------------------------- | :--------------------------------------------------- |
| Initial position     | 1 m radius, equally spaced and facing outward | 5 m radius, equally spaced and oriented clockwise    |
| Normal trajectory    | foward @ 0.33 m/s                             | 10 m diameter CW circle centered on arena @ 0.33 m/s |
| Interactions         | - Bumper: 180° CW<br>  - Top switch: 45° CW   | None                                                 |
| Noise/random         | - Every 20 seconds: 180° CW<br>  - Every 5 seconds, while moving: 0° ≤ angle ≤ 20° CCW  | None       |