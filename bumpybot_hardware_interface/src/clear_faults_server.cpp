//
// Created by Carlos on 9/27/22.
//

#include "ros/ros.h"
#include "ClearFaults.h"

bool clearFaults(bumpybot_hw_interface::ClearFaults::Request  &req,
                 bumpybot_hw_interface::ClearFaults::Response &res)
{
  bool success = true;

  for (unsigned int i = 1; i <= ec_slavecount; i++)
  {
    // clear faults in each Everest servo
    *(ec_slave[i].outputs + 0) = 0x80;    // clear faults

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET * 2);
  }

  // check if all servos are back in OP
  if(ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
    ROS_INFO("Not all servo drives reached OP after clearing faults.")
    success = false
  }
  return success
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_faults_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("clear_faults", clearFaults);
  ROS_INFO("Clearing Faults.");
  ros::spin();

  return 0;
}