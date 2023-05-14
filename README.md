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
ROS 1. To build, clone this repository into your catkin workspace:

```
~ $ cd catkin_ws/src 
~/catkin_ws/src $ git clone --recurse-submodules https://github.com/carlosiglezb/bumpybot.git
```

and then build, either with catkin_make or catkin tools, e.g,

```
~/catkin_ws $ catkin_make  
```

Lastly, don't forget to source:

```
~/catkin_ws $ source devel/setup.bash
```

# Running the simulation

This assumes you have already successfully gone through the 
[Installation](#installation) instructions above and are in your
corresponding `catkin_ws` folder.
We currently have two modes of operation: base velocity, or wheel torque control. 

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

To visualize the robot and control it in RViz, simply run 

```
roslaunch trikey_gazebo gazebo.launch
```
to launch the robot and, on a separate terminal, run

 ```
 roslaunch trikey_base_controller trikey_base_controller.launch
 ``` 

making sure one of the provided rviz files is loaded in the launch file.


## Single Wheel Control Mode

This simply commands wheel velocities independently. To run this mode, simply run 

```
roslaunch trikey_gazebo gazebo.launch
```

This will launch Gazebo and spawn the robot. Now, open a new terminal window and load the controllers

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

# Running the robot

When running the real robot, we often SSH into it, then run (as superuser)

```
roslaunch bumpybot_hw_interface hw_control.launch
```

This initializes the EtherCAT communication and sets the servo drives into
their ENABLE state, ready to use. Additional arguments can be passed on
to run with the camera and/or Lidar ON. Then, on a separate terminal window,
run the base controller

  ```
 roslaunch trikey_base_controller trikey_base_controller.launch
 ``` 

This will, by default, also open up the RViz window with the teleop
plugins loaded.
