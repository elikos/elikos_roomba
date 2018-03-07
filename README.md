# elikos_roomba
ROS package for IARC mission 7 ground &amp; osbtacle robots with on-board Raspberry Pi


`elikos_roomba` works with the [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package to communicate with and drive the Roomba with ROS. Its purpose is to implement the IARC mission 7 ground robot and obstacle [behaviours](#robot-behaviour).

It also has a node to manage the ground robot's top switch and call a service when activated.

<!--
| `elikos_roomba`     |  Topic/service        | Description                                                                                             |
| :------------------ | :-------------------- |:------------------------------------------------------------------------------------------------------- |
|  subscriber         | `bumper`             | Bumper state message (published by `create_autonomy`)                                                   |
|  publisher          | `cmd_vel`            | Drives the robot's wheels according to a forward and angular velocity (`create_autonomy` is subscribed) |
|  service server     | `topswitch_trigger`  | Top switch trigger call (`topswitch_node` is the service client) |
-->


-----


## Prerequisites

1. Raspberry Pi with a RPi-compatible Ubuntu flavour (Ubuntu MATE and Xubuntu 16.04 have been tested; for more, see [here](https://ubuntu-pi-flavour-maker.org/download/)

2. Install [ROS kinetic (Ubuntu/armhf)](http://wiki.ros.org/kinetic/Installation/Ubuntu) (*-desktop*)

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
   git clone https://github.com/USERNAME/elikos_roomba.git
   cd elikos_roomba/
   git remote add upstream https://github.com/elikos/elikos_roomba.git
   ````  
   with `USERNAME` being your GitHub username

3. Clone the [`create_autonomy`](https://github.com/AutonomyLab/create_autonomy) package repo in the `~/roomba_ws/src` folder
   ````
   git clone https://github.com/AutonomyLab/create_autonomy.git
   sudo usermod -a -G dialout $USER
   ````  
   Log out and log back in.

4. Compile
   ````
   cd ~/roomba_ws
   catkin build
   ````
   If compiling for a RPi, use this option to link the wiringPi library and build the topswitch node:
   ````
   catkin build -DRPI=TRUE
   ````
   If you don't have the `catkin_tools` package, you can either install it
   ````
   sudo apt-get install python-catkin-tools
   ````  
   or simply use `catkin_make` instead.
   ````
   catkin_make
   ````  

5. Source 
   ````
   . ./devel/setup.bash
   ````

6. Do stuff

7. Create a pull request


-----


## Launch files

* `rpi_local.launch`  
   * launches `create_autonomy/ca_driver` with a `topswitch_node` inside a `/$(arg robot_type)robot$(arg robot_id)` namespace  
   * to be launched locally on the Raspberry Pi  
   * *arguments*  
      * `robot_id` : unique id of robot [1]
      * `robot_type` : type of robot (`ground` or `obstacle`) [`ground`]

* `robot_ground.launch`  
   * launches `groundrobot_node`  
   * to be launched on a remote computer or on the Raspberry Pi itself  
   * *arguments*  
      * `robot_id` : unique id of robot [1]
      * `pos_x` : initial x position of robot (meters) [0.0]
      * `pos_y` : initial y position of robot (meters) [0.0]
      * `yaw` : initial yaw of robots (radians) [0.0]
      * `color` : color of robot (`red`, `green`, or `white`) [white]

* `robot_obstacle.launch`  
   * launches `obstaclerobot_node`  
   * to be launched on a remote computer or on the Raspberry Pi itself  
   * *arguments*  
      * `robot_id` : unique id of robot [1]
      * `pos_x` : initial x position of robot (meters) [0.0]
      * `pos_y` : initial y position of robot (meters) [0.0]
      * `yaw` : initial yaw of robots (radians) [0.0]

* `robot_sim.launch`  
   * to test `robot_viz`  
   * launches RVIZ, a `serviceredirect_node` and 2 pairs of `groundrobot_node`+`robotviz_node` and a pair of `obstaclerobot_node`+`robotviz_node` inside `/$(arg robot_type)robot$(arg robot_id)` namespaces  

* `service_redirect.launch`  
   * launches a `serviceredirect_node`  
   * *arguments*  
      * `groundrobot_qty` : number of ground robots to manage [1]
      * `obstaclerobot_qty` : number of obstacle robots to manage [1]

* `joy_teleop.launch`  
   * launches a `joy_teleop` node inside a `/$(arg robot_type)robot$(arg robot_id)` namespace  
   * to be launched on a computer with an Xbox 360/Xbox One controller  
   * *arguments*  
      * `robot_id` : unique id of robot to control [1]
      * `robot_type` : type of robot (`ground` or `obstacle`) [`ground`]


<!--
## Nodes

* `groundrobot_node`  
   * ground robot behaviour  
   * *parameters*  
      * `robot_id` : unique id of robot

* `obstaclerobot_node`  
   * obstacle robot behaviour  
   * *parameters*  
      * `robot_id` : unique id of robot

* `topswitch_node`  
   * manages the top switch on the Raspberry Pi through GPIO

* `robotviz_node`  
   * visualize position of robot in `/$(arg robot_type)robot$(arg robot_id)` namespace with RVIZ  
   * *parameters*  
      * `robot_id` : unique id of robot
      * `robot_type` : type of robot (`ground` or `obstacle`)
      * `robot_color` : colour of robot (`red`, `green` or `white`)
      * `init_pos_x` : initial x position of robo (meters)
      * `init_pos_y` : initial y position of robot (meters)
      * `init_pos_z` : initial z position of robot (meters) (should be 0.0)
      * `init_yaw` : initial yaw of robots (radians)

* `serviceredirect_node`  
   * offer global activation/deactivation/toggle services and redirect service calls to all robots inside namespaces (e.g. a call to `/activate` also calls services `/groundrobot1/activate` to `/groundrobot$groundrobot_qty/activate`)  
   * *parameters*  
      * `groundrobot_qty` : number of ground robots to manage
      * `obstaclerobot_qty` : number of obstacle robots to manage

* `arenamanager_node`
   * (not currently working)  
   * checks position estimate of robots (given by their `robotviz_node`) and triggers their bumper if they leave the arena. Can be useful to avoid putting robots back in the arena when testing. Currently only supports ground robots.  
   * *parameters*  
      * `groundrobot_qty` : number of ground robots to manage
      * `arena_dimension` : dimension of (square) arena (meters)
-->

## Services

* `/toggle_activate`  
   * activate/deactivate all robots (via `serviceredirect_node`)

* `/$(arg robot_type)robot$(arg robot_id)/toggle_activate`  
   * activate/deactivate robot with id `$robot_id` and type `$robot_type`
   * *example*
       * For a ground robot with id 1
          ````
          rosservice call /groundrobot1/toggle_activate
          ````

* `/$(arg robot_type)robot$(arg robot_id)/topswitch_trigger`  
   * usually called by `topswitch_node` when top switch of robot with id `$robot_id` is triggered

* `/$(arg robot_type)robot$(arg robot_id)/bumper_trigger`  
   * simulate triggering of bumper of robot with id `$robot_id` and type `$robot_type`

## Using multiple robots

To have multiple robots running on the same `roscore`:  
   * start a `roscore` on a computer (preferably not one of the RPis), and note the IP
   * for every robot `i`, starting from `i=1`
      * on RPi through SSH:
         * set `ROS_MASTER_URI` (pointing to the above IP)
         * set `ROS_IP` (pointing to this RPi)
         * launch `rpi_local.launch` with a unique `robot_id` and a corresponding `robot_type`
             ````
             roslaunch elikos_roomba rpi_local.launch robot_id:=i robot_type:=TYPE
             ````
      * on host computer or RPi through SSH, launch `robot_ground.launch` or `robot_obstacle.launch` with corresponding `robot_id`
          ````
          roslaunch elikos_roomba robot_ground.launch robot_id:=i
          ````
          or
          ````
          roslaunch elikos_roomba robot_obstacle.launch robot_id:=i
          ````
      * or launch `joy_teleop.launch` with corresponding `robot_id` and `robot_type` on a computer with an Xbox controller
          ````
          roslaunch elikos_roomba joy_teleop.launch robot_id:=i robot_type:=TYPE
          ````
   * launch `service_redirect.launch` with the number of ground robots `ng` and number of obstacle robots `no`
       ````
       roslaunch elikos_roomba service_redirect.launch groundrobot_qty:=ng obstaclerobot_qty:=no
       ````
   * activate robots
       ````
       rosservice call /activate
       ````


-----


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
      3. Battery may be dead. Charge the roomba!


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


-----

## Generate documentation

To generate documentation with doxygen (because why not; sometimes a readme isn't enough!)

````
doxygen Doxyfile
```` 

## Robot behaviour

Behaviour description of the ground robot and obstacle robot according to the [official rules](http://www.aerialroboticscompetition.org/rules.php).

| Info                 | Ground robot                                  | Obstacle robot                                       |
| :------------------- |:--------------------------------------------- | :--------------------------------------------------- |
| Initial position     | 1 m radius, equally spaced and facing outward | 5 m radius, equally spaced and oriented clockwise    |
| Normal trajectory    | foward @ 0.33 m/s                             | 10 m diameter CW circle centered on arena @ 0.33 m/s |
| Interactions         | - Bumper: 180° CW<br>  - Top switch: 45° CW   | None                                                 |
| Noise/random         | - Every 20 seconds: 180° CW<br>  - Every 5 seconds, while moving: 0° ≤ angle ≤ 20° CCW  | None       |

## Hardware

Parts list:

* Raspberry Pi 3 (because it has built-in Wi-Fi)
* Power bank (make sure it can output >= 2.5 A per port; [this one](https://www.amazon.ca/dp/B00Z9QVE4Q/) works)
* Power cable (using a short cable like [these](https://www.amazon.ca/dp/B01C5N6D96/) is probably a good idea)  
  Tip: disconnect the RPi power cable when not using the robot. That seems to drain the power bank very quickly even if the RPi is turned off.

