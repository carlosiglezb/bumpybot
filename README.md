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

# Dependencies

This stack has mostly `ros_control` dependencies. If you have already done a full ROS
install, chances are that you have already installed most of them. If not,
you may need to make sure you install the following for your particular
`<distro>`:

```
sudo apt-get install ros-<distro>-gazebo-ros-control ros-<distro>-effort-controllers 
ros-<distro>-ros-controllers ros-<distro>-ros-control ros-<distro>-gazebo-ros-pkgs
ros-<distro>-controller-manager ros-<distro>-hardware-interface
```


# Installation

This stack has been developed (and only tested) on Ubuntu 18.04 LTS and
ROS 1. To build, clone this repository into your catkin workspace and make
sure all submodules are on the main branch:

```
~ $ cd catkin_ws
~/catkin_ws $ git clone --recurse-submodules https://github.com/carlosiglezb/bumpybot.git src
~/catkin_ws $ cd src && git submodule foreach 'git checkout main'
```

and then build, either with catkin_make or catkin tools, e.g,

```
~/catkin_ws $ catkin_make  
```

Lastly, don't forget to source:

```
~/catkin_ws $ source devel/setup.bash
```

# Setup

If you intend to use the BumpyBot in Gazebo simulation,  refer to the following package:
* [trikey_gazebo](https://github.com/carlosiglezb/trikey_gazebo/tree/main)


## Robot Bootup
Turn on the swiches for Battery, Logic, and Motor.
If the local computer is not turned on, manually boot up the computer.
Set up your network settings to connect to the robot.

For wireless connection, make sure to conenct your remote computer to the hcrlab2 wireless network.
You may access the BumpyBot computer by `ssh hcrl-bumpybot@192.168.52.35`

## Robot Connection
You can access the BumpyBot computer through SSH either from ethernet or WiFi.

For ethernet connection, you need to confguire your IPv4 addressess to be in the same subnet as the robot.

For wireless connection, make sure to conenct your remote computer to the hcrlab2 wireless network.
You may access the BumpyBot computer by `ssh hcrl-bumpybot@192.168.52.35`

## ROS Network Setup
If you are hosting `roscore` on you remote computer, you need to set the ROS_MASTER_URI on both your remote computer and the BumpyBot computer.
This configuration can be set in the `.bashrc` file.

## Robot Launch
In order to launch the robot, you need to run the following commands on the BumpyBot computer.
```
sudo su
```
This command will log you into root with the root environment. This is necessary to run the motors

In the root enviroment, ROS packages may not be located automatically, access to the `bumpybot_hardward_interface` by following command
```
cd ~/bumpybot_ws/src/bumpybot_hardware_interface/launch
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
You can send velocity commands by gui.


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
sudo apt-get install ros-<distro>-teleop-twist-keyboard
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
