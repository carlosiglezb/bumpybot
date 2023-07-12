# BumpyBot Simulation Instructions

## Running the simulation

This assumes you have already successfully gone through the 
[Installation](./README.md#installation) instructions above and are in your
corresponding `catkin_ws` folder.
We currently have two modes of operation: base velocity, or wheel torque control. 

## Gazebo and RViz
To visualize the robot and control it in RViz, simply run 

```
roslaunch trikey_gazebo gazebo.launch
```