#ifndef BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP
#define BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace bumpybot_torque_contact {

class EncoderFilter : public nodelet::Nodelet
{
public:
  EncoderFilter();

private:
  virtual void onInit();

  // Example: a timer and a publisher
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher  debug_pub_;
  ros::Timer      timer_;

  void timerCallback(const ros::TimerEvent &event);
};

}  // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP
