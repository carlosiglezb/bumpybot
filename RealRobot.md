# BumpyBot Real Robot Instructions

## Running the robot

This assumes you have already successfully gone through the 
[Installation](./README.md#installation) instructions above and are in your
corresponding `catkin_ws` folder.
We currently have two modes of operation: base velocity, or wheel torque control. 

## Instructions

When running the real robot, we often SSH into it and run as superuser: \
** Note: In order for BumpyBot to connect you need to disconnect the motor 
ethernet port when connecting the computer to BumpyBot.

```
ssh hcrl-bumpybot@192.168.2.3       #Example HOSTNAME@IP.ADDRESS
sudo su
```


Then run the hw interface on the robot: \
**Note: _Remember to add the ros network configuation settings to the `.bashrc` of 
the superuser. Alternatively you may also run: `source bbot.setup`._ \
**Note: _If you disconnected the motor ethernet port reconnect the cable to continue._ \
**Note: _If `Slave 1 State=  12 StatusCode=  24 : Invalid input mapping` occurs and motors turn on, ctrl-C and relaunch hw interface_
 
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


## Extra

If tab completion is not working, try adding the package to `ROS_PACKAGE_PATH` 
with:
```
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
```
