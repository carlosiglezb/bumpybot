#include "bumpybot_torque_contact/encoder_filter.hpp"
#include <pluginlib/class_list_macros.h>

namespace bumpybot_torque_contact {

EncoderFilter::EncoderFilter()
  : nh_(), private_nh_("~")
{
}

void EncoderFilter::onInit()
{
  nh_         = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  debug_pub_ = nh_.advertise<std_msgs::String>("encoder_filter_debug", 1);

  // Simple test timer
  timer_ = nh_.createTimer(
      ros::Duration(0.5),
      &EncoderFilter::timerCallback,
      this);

  NODELET_INFO("EncoderFilter nodelet initialized");
}

void EncoderFilter::timerCallback(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "EncoderFilter test tick";
  debug_pub_.publish(msg);
}

}  // namespace bumpybot_torque_contact

// Register as plugin
PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::EncoderFilter, nodelet::Nodelet)
