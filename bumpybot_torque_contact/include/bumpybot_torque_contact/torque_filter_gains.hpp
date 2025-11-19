#ifndef BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP
#define BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace bumpybot_torque_contact {

class TorqueFilterGains : public nodelet::Nodelet
{
public:
  TorqueFilterGains();

private:
  virtual void onInit();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher  debug_pub_;
  ros::Timer      timer_;

  void timerCallback(const ros::TimerEvent &event);
};

}  // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP
