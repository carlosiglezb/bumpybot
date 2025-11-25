#include "bumpybot_torque_contact/torque_filter_gains.hpp"

#include <boost/bind/bind.hpp>

namespace bumpybot_torque_contact
{

TorqueFilterGains::TorqueFilterGains()
  : nh_(),
    private_nh_("~"),
    enable_downsampling_(false),
    downsample_factor_(5),
    enable_low_pass_(true),
    low_pass_window_size_(120),
    offset_window_duration_(3.0),
    T_MIN_(0.0),
    t_start_(-1.0),
    zeroing_complete_(false),
    downsample_counter_(0)
{
  // Default: identity 3×3
  torque_gain_[0] = { {1.0, 0.0, 0.0} };
  torque_gain_[1] = { {0.0, 1.0, 0.0} };
  torque_gain_[2] = { {0.0, 0.0, 1.0} };
  perm_[0] = 2;
  perm_[1] = 1; 
  perm_[2] = 0; 
}

void TorqueFilterGains::onInit()
{
  nh_         = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  NODELET_INFO("TorqueFilterGains nodelet initializing...");

  // Optional: initial params from param server (they'll be overridden by dynreconf)
  private_nh_.param("enable_downsampling",  enable_downsampling_,  enable_downsampling_);
  private_nh_.param("downsample_factor",    downsample_factor_,    downsample_factor_);
  private_nh_.param("enable_low_pass",      enable_low_pass_,      enable_low_pass_);
  private_nh_.param("low_pass_window_size", low_pass_window_size_, low_pass_window_size_);
  private_nh_.param("zeroing_duration",     offset_window_duration_, offset_window_duration_);
  private_nh_.param("T_MIN",                T_MIN_, T_MIN_);
  private_nh_.param("perm0",                perm_[0], perm_[0]);
  private_nh_.param("perm1",                perm_[1], perm_[1]);
  private_nh_.param("perm2",                perm_[2], perm_[2]);

  // Publisher / subscriber / service
  filtered_pub_ = nh_.advertise<sensor_msgs::JointState>(
      "/filtered_torque_data", 100);

  torque_sub_ = nh_.subscribe(
      "/torque_sensor_v", 100,
      &TorqueFilterGains::torqueCallback, this);

  reset_service_ = nh_.advertiseService(
      "reset_offsets",
      &TorqueFilterGains::resetOffsets,
      this);

  // dynamic_reconfigure server
  dr_srv_.reset(new dynamic_reconfigure::Server<TorqueDataFilterConfig>(private_nh_));
  dynamic_reconfigure::Server<TorqueDataFilterConfig>::CallbackType cb;
  cb = boost::bind(&TorqueFilterGains::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
  dr_srv_->setCallback(cb);

  NODELET_INFO("TorqueFilterGains nodelet initialized.");
}

void TorqueFilterGains::configCallback(TorqueDataFilterConfig &config, uint32_t /*level*/)
{
  enable_downsampling_    = config.enable_downsampling;
  downsample_factor_      = config.downsample_factor;
  enable_low_pass_        = config.enable_low_pass;
  low_pass_window_size_   = config.low_pass_window_size;
  offset_window_duration_ = config.zeroing_duration;
  T_MIN_                  = config.T_MIN;
  // permutation
  perm_[0] = config.perm0;
  perm_[1] = config.perm1;
  perm_[2] = config.perm2;
  // sanity check – make sure all three are distinct 0..2
  bool ok = true;
  for (int i = 0; i < 3; ++i)
  {
    if (perm_[i] < 0 || perm_[i] > 2) ok = false;
  }
  if (perm_[0] == perm_[1] || perm_[0] == perm_[2] || perm_[1] == perm_[2])
    ok = false;

  if (!ok)
  {
    NODELET_WARN("Invalid permutation (%d,%d,%d); falling back to (2,0,1)",
                 perm_[0], perm_[1], perm_[2]);
    perm_[0] = 2; perm_[1] = 0; perm_[2] = 1;
  }
  // 3×3 gain matrix from dynamic reconfigure
  // You need to define these in TorqueDataFilter.cfg, e.g.:
  //   double m00, m01, ..., m22
  updateGainMatrixFromConfig(config);

  NODELET_INFO_STREAM("Reconfigure: downsample=" << (enable_downsampling_ ? "true" : "false")
                    << " factor=" << downsample_factor_
                    << ", low_pass=" << (enable_low_pass_ ? "true" : "false")
                    << " lpw=" << low_pass_window_size_
                    << ", offset_window=" << offset_window_duration_);
}

void TorqueFilterGains::updateGainMatrixFromConfig(const TorqueDataFilterConfig &config)
{
  // Assuming you added these fields to TorqueDataFilter.cfg:
  //   m00 m01 m02
  //   m10 m11 m12
  //   m20 m21 m22
  torque_gain_[0][0] = config.m00;
  torque_gain_[0][1] = config.m01;
  torque_gain_[0][2] = config.m02;

  torque_gain_[1][0] = config.m10;
  torque_gain_[1][1] = config.m11;
  torque_gain_[1][2] = config.m12;

  torque_gain_[2][0] = config.m20;
  torque_gain_[2][1] = config.m21;
  torque_gain_[2][2] = config.m22;
}

bool TorqueFilterGains::resetOffsets(std_srvs::Trigger::Request &,
                                     std_srvs::Trigger::Response &res)
{
  offset_buffers_.clear();
  offsets_.clear();
  raw_buffer_.clear();
  low_pass_buffers_.clear();

  t_start_ = -1.0;
  zeroing_complete_ = false;

  NODELET_INFO("Offsets and zeroing state have been reset.");

  res.success = true;
  res.message = "Offsets reset successfully.";
  return true;
}

void TorqueFilterGains::torqueCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  const double t = ros::Time::now().toSec();

  // First callback: mark start time
  if (t_start_ < 0.0)
  {
    t_start_ = t;
    NODELET_INFO_STREAM("Starting " << offset_window_duration_
                        << "-second zeroing period...");
  }

  // Downsampling
  if (enable_downsampling_)
  {
    ++downsample_counter_;
    if (downsample_counter_ % downsample_factor_ != 0)
      return;
  }

  // Assume 3 DOF, remap [0,1,2] → [2,0,1] like the Python code
  if (msg->name.size() < 3 || msg->position.size() < 3)
  {
    NODELET_WARN_THROTTLE(1.0, "Received JointState with <3 elements; ignoring.");
    return;
  }

  std::vector<std::string> orig_names = msg->name;  // preserve original order
 // const int perm[3] = {2, 0, 1};

  std::vector<std::string> names(3);
  std::array<double, 3> raw;

  for (int i = 0; i < 3; ++i)
  {
    const int idx = perm_[i];  // now configurable
    if (idx < 0 || idx >= static_cast<int>(msg->name.size()))
    {
      NODELET_ERROR_THROTTLE(1.0,
          "Permutation index %d out of range for incoming JointState (size=%zu)",
          idx, msg->name.size());
      return;
    }
    names[i] = msg->name[idx];
    raw[i]   = msg->position[idx];
  }

  // Update sliding raw_buffer for potential future use
  raw_buffer_.push_back(std::make_pair(t, raw));
  const double cutoff = t - offset_window_duration_;
  while (!raw_buffer_.empty() && raw_buffer_.front().first < cutoff)
  {
    raw_buffer_.pop_front();
  }

  // Initial zeroing window
  if (!zeroing_complete_)
  {
    if ((t - t_start_) <= offset_window_duration_)
    {
      // Collect offsets
      for (int i = 0; i < 3; ++i)
      {
        std::vector<double> &buf = offset_buffers_[names[i]];
        buf.push_back(raw[i]);
      }

      // Publish all-zero during window
      sensor_msgs::JointState out;
      out.header   = msg->header;
      out.name     = orig_names;
      out.position.assign(orig_names.size(), 0.0);
      filtered_pub_.publish(out);
      return;
    }
    else
    {
      // Compute final offsets (mean)
      NODELET_INFO("Zeroing period complete. Calculating final offsets...");
      for (int i = 0; i < 3; ++i)
      {
        const std::string &name = names[i];
        std::vector<double> &buf = offset_buffers_[name];
        double mean = 0.0;
        if (!buf.empty())
        {
          double sum = 0.0;
          for (double v : buf) sum += v;
          mean = sum / static_cast<double>(buf.size());
        }
        offsets_[name] = mean;
        NODELET_INFO_STREAM("Final offset for " << name << ": " << mean);
      }
      offset_buffers_.clear();
      zeroing_complete_ = true;
      NODELET_INFO("Zeroing complete. Offset buffers cleared.");
    }
  }

  // Apply offsets
  std::array<double, 3> corrected;
  for (int i = 0; i < 3; ++i)
  {
    double off = 0.0;
    std::map<std::string, double>::const_iterator it = offsets_.find(names[i]);
    if (it != offsets_.end())
      off = it->second;
    corrected[i] = raw[i] - off;
  }

  // 3×3 gain: scaled = gain * corrected
  std::array<double, 3> scaled;
  for (int i = 0; i < 3; ++i)
  {
    scaled[i] = 0.0;
    for (int j = 0; j < 3; ++j)
    {
      scaled[i] += torque_gain_[i][j] * corrected[j];
    }
  }

  // Low-pass boxcar
  if (enable_low_pass_)
  {
    for (int i = 0; i < 3; ++i)
    {
      std::deque<double> &buf = low_pass_buffers_[names[i]];
      buf.push_back(scaled[i]);
      // enforce window length
      while (static_cast<int>(buf.size()) > low_pass_window_size_)
        buf.pop_front();

      double sum = 0.0;
      for (double v : buf) sum += v;
      if (!buf.empty())
        scaled[i] = sum / static_cast<double>(buf.size());
    }
  }

  // Minimum threshold
  for (int i = 0; i < 3; ++i)
  {
    if (std::fabs(scaled[i]) < T_MIN_)
      scaled[i] = 0.0;
  }

  // Publish filtered result in original name order
  sensor_msgs::JointState out;
  out.header = msg->header;
  out.name   = orig_names;

  // We only computed 3 torques, but message could have more names.
  // Here we mirror the Python behavior: overwrite positions for indices 0..2
  // with the permuted+filtered values, keep others unchanged if present.
  out.position = msg->position;
  for (int i = 0; i < 3 && i < static_cast<int>(out.position.size()); ++i)
  {
    int idx = perm_[i];
    out.position[idx] = scaled[i];
  }

  filtered_pub_.publish(out);
}

// Register nodelet
}  // namespace bumpybot_torque_contact

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::TorqueFilterGains,
                       nodelet::Nodelet)
