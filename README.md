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
Additional setup steps must be taken depending on if you are using simulation 
or the real robot. Go to their respective files to see the instructions:
* [Real Robot Setup](./RealRobot.md)
* [Simulation Setup](./Simulation.md)


# Running the Robot

## Base Velocity Control Mode

This can be used to command linear and angular velocities (e.g., through `cmd_vel`) 
of the base of the robot. The 
[base_controller](https://github.com/carlosiglezb/trikey_base_controller) 
package then computes the corresponding wheel 
velocities to achieve the desired twist. For instance, these commands can be
sent for teleop via the `teleop-twist-keyboard` package, or via our
provided [RViz plugin](https://github.com/carlosiglezb/bumpybot_teleop). 

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
roslaunch trikey_base_controller trikey_base_controller.launch
```


### Using the provided teleop package

 Run

 ```
 roslaunch trikey_base_controller trikey_base_controller.launch
 ``` 

making sure one of the provided rviz files is loaded in the launch file.


## Single Wheel Control Mode

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
