#include "bumpybot_torque_contact/encoder_filter.hpp"
#include <pluginlib/class_list_macros.h>

#include <algorithm>  // std::find
#include <numeric>    // std::accumulate
#include <cmath>      // std::abs

namespace bumpybot_torque_contact
{

EncoderFilter::EncoderFilter()
  : nh_(), private_nh_("~"),
    enable_low_pass_(false),
    enable_downsampling_(false),
    downsample_factor_(1),
    low_pass_window_size_(50),
    T_MIN_(0.0),
    downsample_counter_(0)
{
  // Canonical wheel names
  wheel_names_ = {"wheel0_joint", "wheel1_joint", "wheel2_joint"};

  // Default permutation: 2,1,0 as you said you settled on "210"
  perm_[0] = 2;
  perm_[1] = 1;
  perm_[2] = 0;
}

void EncoderFilter::onInit()
{
  NODELET_INFO("EncoderFilter nodelet starting...");

  nh_         = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>(
      "/joint_states", 10, &EncoderFilter::jointCallback, this);

  filtered_pub_ = nh_.advertise<sensor_msgs::JointState>(
      "/filtered_joint_states", 10);

  // dynamic reconfigure
  dr_server_.reset(new DRServer(private_nh_));
  DRServer::CallbackType cb =
      boost::bind(&EncoderFilter::configCallback, this, _1, _2);
  dr_server_->setCallback(cb);

  resetBuffers();

  NODELET_INFO("EncoderFilter nodelet initialized.");
}

void EncoderFilter::resetBuffers()
{
  pos_buf_.clear();
  vel_buf_.clear();
  eff_buf_.clear();
}

void EncoderFilter::configCallback(EncoderFilterConfig &config, uint32_t)
{
  enable_low_pass_      = config.enable_low_pass;
  enable_downsampling_  = config.enable_downsampling;
  downsample_factor_    = config.downsample_factor;
  low_pass_window_size_ = config.low_pass_window_size;
  T_MIN_                = config.T_MIN;

  // permutation from EncoderFilter.cfg
  perm_[0] = config.perm0;
  perm_[1] = config.perm1;
  perm_[2] = config.perm2;

  bool ok = true;
  for (int i = 0; i < 3; ++i)
  {
    if (perm_[i] < 0 || perm_[i] > 2)
      ok = false;
  }
  if (perm_[0] == perm_[1] || perm_[0] == perm_[2] || perm_[1] == perm_[2])
    ok = false;

  if (!ok)
  {
    NODELET_WARN("EncoderFilter: invalid permutation (%d,%d,%d); "
                 "falling back to (2,1,0)",
                 perm_[0], perm_[1], perm_[2]);
    perm_[0] = 2;
    perm_[1] = 1;
    perm_[2] = 0;
  }

  NODELET_INFO("EncoderFilter reconfigure: "
               "low_pass=%d win=%d  downsample=%d factor=%d  T_MIN=%.3f  perm=(%d,%d,%d)",
               enable_low_pass_, low_pass_window_size_,
               enable_downsampling_, downsample_factor_, T_MIN_,
               perm_[0], perm_[1], perm_[2]);

  // Filtering buffers depend on config; safest is to clear them
  resetBuffers();
}

void EncoderFilter::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // downsampling
  if (enable_downsampling_)
  {
    if (++downsample_counter_ % downsample_factor_ != 0)
      return;
  }

  // Prepare output
  sensor_msgs::JointState out;
  out.header  = msg->header;
  out.name    = msg->name;
  out.position = msg->position;
  out.velocity = msg->velocity;
  out.effort   = msg->effort;

  // NOTE on permutation:
  // For this encoder filter, all three wheel joints are treated identically
  // (same low-pass window, same threshold). So perm_ does NOT change the
  // math right now. If you later give each channel different behavior,
  // you can use perm_ to map which wheel is considered "channel 0/1/2".
  //
  // Here we still filter by joint *name* exactly as before.

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    const std::string &name = msg->name[i];

    double p = msg->position[i];
    double v = msg->velocity[i];
    double e = msg->effort[i];

    // only filter our wheel joints
    bool is_wheel =
        std::find(wheel_names_.begin(), wheel_names_.end(), name)
        != wheel_names_.end();

    if (!is_wheel)
    {
      // leave as-is
      continue;
    }

    // ---- low pass (boxcar) ----
    if (enable_low_pass_)
    {
      std::deque<double> &pb = pos_buf_[name];
      std::deque<double> &vb = vel_buf_[name];
      std::deque<double> &eb = eff_buf_[name];

      pb.push_back(p);
      vb.push_back(v);
      eb.push_back(e);

      while (static_cast<int>(pb.size()) > low_pass_window_size_) pb.pop_front();
      while (static_cast<int>(vb.size()) > low_pass_window_size_) vb.pop_front();
      while (static_cast<int>(eb.size()) > low_pass_window_size_) eb.pop_front();

      p = std::accumulate(pb.begin(), pb.end(), 0.0) / pb.size();
      v = std::accumulate(vb.begin(), vb.end(), 0.0) / vb.size();
      e = std::accumulate(eb.begin(), eb.end(), 0.0) / eb.size();
    }

    // ---- threshold ----
    if (std::abs(p) < T_MIN_) p = 0.0;
    if (std::abs(v) < T_MIN_) v = 0.0;
    if (std::abs(e) < T_MIN_) e = 0.0;

    out.position[i] = p;
    out.velocity[i] = v;
    out.effort[i]   = e;
  }

  filtered_pub_.publish(out);
}

} // namespace bumpybot_torque_contact

PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::EncoderFilter,
                       nodelet::Nodelet)
