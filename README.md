# Bumpybot

Bumpybot is an omniwheel robot with fairly strong motors and with vision and 
force sensing capabilities. It has been designed to be 
(a) a research platform to benchmark navigation algorithms using
both visual and force-sensing measurements, for instance,
in crowded environments; and (b) a mobility device robot, i.e.,
one capable of carrying a person. 

This stack contains our low-level code, which uses both ROS,
[ros_control](http://wiki.ros.org/ros_control), and
SOEM. Although not perfect, we hope it helps others more easily
develop and integrate these tools for their own robots. 
Currently, we are using the following:

* Maxon EC45 BLDC motors
* HiTec rotary torque sensors
* US digital E6 optical encoders
* Everest XCR servi drives (EtherCAT) 
* UP Xtreme computer (patched with RT_PREEMPT)
* Azure Kinect
* RP Lidar 2D (A2M8)

![real-bumpybot-move-right](https://github.com/carlosiglezb/bumpybot/assets/97119009/00107578-1676-4b36-a3f5-226d36a4ed69)



# Installation

Install [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu 20.04 (recommended)

Install [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) for Ubuntu 18.04

## ROS Packages Dependencies [TODO: Change into `rosdep install`]



<!-- sudo rosdep init -->


<!-- rosdep install --from-paths src --ignore-src -r -y -->
```
sudo apt-get install \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-ros-controllers \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-soem \
    ros-$ROS_DISTRO-rosparam-shortcuts \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-teb-local-planner \
    ros-$ROS_DISTRO-rtabmap-ros \
    python3-rosdep

```


This stack has been developed (and only tested) on Ubuntu 18.04 LTS & Ubuntu 20.04 and
ROS 1. To build, clone this repository into your catkin workspace and make
sure all submodules are on the main branch:

```
cd catkin_ws
git clone --recurse-submodules https://github.com/carlosiglezb/bumpybot.git src
cd src && git submodule foreach 'git checkout main'
```

and then build, either with catkin_make or catkin tools, e.g:

```
cd ~/catkin_ws
catkin_make  
```

Lastly, don't forget to source:

```
cd ~/catkin_ws
source devel/setup.bash
```

# Setup

If you intend to use the BumpyBot in Gazebo simulation,  refer to the following package:
* [trikey_gazebo](https://github.com/carlosiglezb/trikey_gazebo/tree/main)


## Robot Bootup
Turn on the switches for Battery, Logic, and Motor.
If the local computer is not turned on, manually boot up the computer.
Set up your network settings to connect to the robot.

For a wireless connection, make sure to connect your remote computer to the hcrlab2 wireless network.
You may access the BumpyBot computer by `ssh hcrl-bumpybot@192.168.52.35`

## Robot Connection
You can access the BumpyBot computer wirelessly/wired via SSH or via NoMachine Remote Desktop

### Remote Desktop

[Click here for instructions on setting up and using NoMachine for a remote desktop environment.](nomachine_instructions.md)

### Ethernet Connection
For an ethernet connection, you need to confirm your IPv4 addresses to be in the same subnet as the robot.


### Wireless Connection: bumpybot WiFi (Default)
When BumpyBot computer turn on, it hosts an access point `bumpybot` by default
Connect via ssh command: `ssh hcrl-bumpybot@10.42.99.99`

If `bumpybot` wifi ssid isn't being advertised, run `sudo hotspot_mode.sh` (or reboot BumbyBot if you don't already have terminal access). note that the `wifi_mode.sh` and `hotspot_mode.sh` scripts live in `/usr/local/bin/`.

Note that BumpyBot computer may not be able to connect to the internet in this mode.



### Wireless Connection via hcrlab2 WiFi network

On BumpyBot computer run
`sudo wifi_mode.sh`

For a wireless connection, make sure to connect your remote computer to the hcrlab2 wireless network.
You may access the BumpyBot computer by `ssh hcrl-bumpybot@192.168.50.35`

This will disable the hotspot if already running.


### Camera Launch
To launch the camera, run the following command on the BumpyBot computer (No sudo su)
```
roslaunch azure_kinect_ros_driver driver.launch
```

to run as nodelet, run
```
roslaunch azure_kinect_ros_driver kinect_rgbd.launch
```

to launch with `trikey_base_controller`
```
roslaunch bumpyboy_navigation depth_nav.launch
```


## ROS Network Setup
If you are hosting `roscore` on you remote computer, you need to set the ROS_MASTER_URI on both your remote computer and the BumpyBot computer.
This configuration can be set in the `.bashrc` file.

## Robot Launch
In order to launch the robot, you need to run the following commands on the BumpyBot computer.
```
sudo su
```
This command will log you into root with the root environment. This is necessary to run the motors

In the root environment, ROS packages may not be located automatically, access to the `bumpybot_hardward_interface` by the following command
```
cd /home/hcrl-bumpybot/bumpybot_ws/src/bumpybot_hardware_interface/launch
```
Then run the following command to launch the robot
```
roslaunch hw_control.launch
```

You may see the following error message
```
Slave 1 State=  12 StatusCode=  24 : Invalid input mapping` 
```

Kill the process by `ctrl-C` and relaunch it.

## Launch Camera
`roslaunch azure_kinect_ros`

## Robot Manual Control

### Xbox Controller
To control the robot with an Xbox controller, run the following command
```
roslaunch trikey_base_controller trikey_base_controller.launch
```

Press and hold RB or LB to move the robot with joystick and D-pad.

### RViz Teleop Gui Control
To control the robot with RViz Teleop Gui, run the following command
```
roslaunch trikey_base_controller trikey_base_controller.launch xbox:=false
```
You can send velocity commands by GUI.


<!-- This can be used to command linear and angular velocities (e.g., through `cmd_vel`) 
of the base of the robot. The 
[base_controller](https://github.com/carlosiglezb/trikey_base_controller) 
package then computes the corresponding wheel 
velocities to achieve the desired twist. For instance, these commands can be
sent for teleop via the `teleop-twist-keyboard` package, or via our
provided [RViz plugin](https://github.com/carlosiglezb/bumpybot_teleop).  -->

### Using teleop-twist-keyboard

Install the package through

```
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
```

Then, on one terminal window run

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=trikey/base_controller/cmd_vel
```

and on another one run the keyboard controller

```
roslaunch trikey_base_controller trikey_base_controller.launch xbox:=false
```



### Single Wheel Control Mode

This simply commands wheel velocities independently. Open a new terminal window and load the controllers

```
roslaunch trikey_control trikey_control.launch
```

On a new terminal window, you should be able to see all of the topics published by the controllers

```
rostopic list
```

Try setting the commanded wheel position (in radians) via the ROS topic `wheel<number>_position_controller`, e.g.,

```
rostopic pub -1 /trikey/wheel1_position_controller/command std_msgs/Float64 "data: 1"
```

## Extra

If tab completion is not working, try adding the package to `ROS_PACKAGE_PATH` 
with:
```
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
```

To test if motors are functional without ROS run these line.

```
cd ~/ros/hcrl_core/soem/install/bin
```

```
sudo ./everest_test enp1s0
```


## Robot Navigation

refer to [bumybot_navigation](https://github.com/dokkev/bumpybot_navigation) package for more information.

# Publication

This code base is associated to the following publication, please feel free
to check it out: 

``` latex
@proceedings{bumpybot,
    author = {Gonzalez, Carlos and Lee, Samantha and Montano, Francisco and Ortega, Steven and Kang, Dong Ho and Jaiswal, Mehar and Jiao, Junfeng and Sentis, Luis},
    title = {Design of a Person-Carrying Robot for Contact Compliant Navigation},
    series = {International Design Engineering Technical Conferences and Computers and Information in Engineering Conference},
    year = {2023},
    month = {08}
}
```
All feedback is welcome!
