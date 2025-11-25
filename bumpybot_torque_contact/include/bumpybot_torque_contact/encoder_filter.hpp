#ifndef BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP
#define BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <bumpybot_torque_contact/EncoderFilterConfig.h>

#include <deque>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace bumpybot_torque_contact
{

class EncoderFilter : public nodelet::Nodelet
{
public:
  EncoderFilter();

private:
  virtual void onInit();

  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void resetBuffers();

  // dynamic reconfigure callback
  void configCallback(EncoderFilterConfig &config, uint32_t level);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ROS interface
  ros::Subscriber joint_sub_;
  ros::Publisher  filtered_pub_;

  // Filtering state
  bool   enable_low_pass_;
  bool   enable_downsampling_;
  int    downsample_factor_;
  int    low_pass_window_size_;
  double T_MIN_;   // threshold

  int downsample_counter_;

  // buffers: name → deque of values
  std::map<std::string, std::deque<double>> pos_buf_;
  std::map<std::string, std::deque<double>> vel_buf_;
  std::map<std::string, std::deque<double>> eff_buf_;

  // joint names we consider "wheels"
  std::vector<std::string> wheel_names_;

  // permutation (same idea as torque filter)
  int perm_[3];

  // dynamic reconfigure server
  typedef dynamic_reconfigure::Server<bumpybot_torque_contact::EncoderFilterConfig> DRServer;
  boost::shared_ptr<DRServer> dr_server_;
};

} // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_ENCODER_FILTER_HPP
