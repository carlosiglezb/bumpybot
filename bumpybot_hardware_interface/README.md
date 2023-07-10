# Bumpybot Hardware Interface 

ROS [hardware interface](http://docs.ros.org/en/melodic/api/hardware_interface/html/c++/index.html) 
EtherCAT implementation on the omniwheel robot Bumpybot. This package uses
SOEM along with ros_control and has been inspired by other great references, such
as [ros_ethercat](https://github.com/ShunyaoWang/ros_ethercat/blob/master/ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.cpp),
the [diffbot_hw_interface](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_base/include/diffbot_base/diffbot_hw_interface.h),
the [ethercat_interface](https://github.com/nobleo/ethercat_interface/blob/develop/src/ethercat_interface.cpp), and 
PikNikRobitics' [ros control boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate).  

This has been only tested in ROS Melodic. I hope others find it useful in their
own projects using ROS and SOEM. 

# Publication
If you are interested in the platform using this code, or find this
useful and would like to cite our work, please consider citing:

``` latex
@proceedings{bumpybot,
    author = {Gonzalez, Carlos and Lee, Samantha and Montano, Francisco and Ortega, Steven and Kang, Dong Ho and Jaiswal, Mehar and Jiao, Junfeng and Sentis, Luis},
    title = {Design of a Person-Carrying Robot for Contact Compliant Navigation},
    series = {International Design Engineering Technical Conferences and Computers and Information in Engineering Conference},
    year = {2023},
    month = {08}
}
```
