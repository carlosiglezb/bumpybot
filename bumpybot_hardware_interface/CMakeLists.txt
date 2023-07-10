cmake_minimum_required(VERSION 3.0.2)
project(bumpybot_hw_interface)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    soem
    roscpp
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    urdf
    message_generation
#    joint_limits_interface
#    transmission_interface
)

add_service_files(
	FILES
	ClearFaults.srv
)

add_message_files(
	FILES
	ServoTemp.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS controller_manager hardware_interface roscpp rosparam_shortcuts sensor_msgs std_msgs urdf message_runtime
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
        src/generic_hw_control_loop.cpp
        src/bumpybot_hw_interface.cpp
#        src/bumpybot_hw_main.cpp
)

add_executable(bumpybot_hw_main src/bumpybot_hw_main.cpp src/generic_hw_control_loop.cpp)
target_link_libraries(bumpybot_hw_main bumpybot_hw_interface_lib ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
