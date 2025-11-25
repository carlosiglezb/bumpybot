#ifndef BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP
#define BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <bumpybot_torque_contact/TorqueDataFilterConfig.h>

#include <deque>
#include <map>
#include <string>
#include <vector>
#include <array>

#include <boost/shared_ptr.hpp>

namespace bumpybot_torque_contact
{

class TorqueFilterGains : public nodelet::Nodelet
{
public:
  TorqueFilterGains();

private:
  // Nodelet interface
  virtual void onInit();

  // Dynamic reconfigure callback
  void configCallback(TorqueDataFilterConfig &config, uint32_t level);

  // Service callback
  bool resetOffsets(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);

  // Main torque callback
  void torqueCallback(const sensor_msgs::JointState::ConstPtr &msg);

  // Helpers
  void updateGainMatrixFromConfig(const TorqueDataFilterConfig &config);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ROS interfaces
  ros::Publisher  filtered_pub_;
  ros::Subscriber torque_sub_;
  ros::ServiceServer reset_service_;

  // dynamic_reconfigure server
  boost::shared_ptr< dynamic_reconfigure::Server<TorqueDataFilterConfig> > dr_srv_;

  // Parameters (runtime)
  bool   enable_downsampling_;
  int    downsample_factor_;
  bool   enable_low_pass_;
  int    low_pass_window_size_;
  double offset_window_duration_;
  double T_MIN_;

  // Time / zeroing state
  double t_start_;          // first callback time (sec), <0 means "unset"
  bool   zeroing_complete_;
  int    downsample_counter_;

  // Buffers / state
  // raw_buffer_: (time, [τ0, τ1, τ2]) – mostly for future debug/extension
  std::deque< std::pair<double, std::array<double, 3> > > raw_buffer_;

  // name → vector of samples (for the initial zeroing window)
  std::map<std::string, std::vector<double> > offset_buffers_;
  // name → final offset
  std::map<std::string, double> offsets_;
  // name → low-pass window
  std::map<std::string, std::deque<double> > low_pass_buffers_;

  // 3×3 gain matrix (row-major: gain[row][col])
  std::array< std::array<double, 3>, 3 > torque_gain_;

    // dynamic permutation for torque channels
   int perm_[3]; 
};

}  // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_TORQUE_FILTER_GAINS_HPP
