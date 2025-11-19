#include "bumpybot_torque_contact/torque_filter_gains.hpp"
#include <pluginlib/class_list_macros.h>

namespace bumpybot_torque_contact {

TorqueFilterGains::TorqueFilterGains()
  : nh_(), private_nh_("~")
{
}

void TorqueFilterGains::onInit()
{
  nh_         = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  debug_pub_ = nh_.advertise<std_msgs::String>("torque_filter_gains_debug", 1);

  // Simple test timer
  timer_ = nh_.createTimer(
      ros::Duration(0.5),
      &TorqueFilterGains::timerCallback,
      this);

  NODELET_INFO("TorqueFilterGains nodelet initialized");
}

void TorqueFilterGains::timerCallback(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "TorqueFilterGains test tick";
  debug_pub_.publish(msg);
}

}  // namespace bumpybot_torque_contact

// Register as plugin
PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::TorqueFilterGains, nodelet::Nodelet)
