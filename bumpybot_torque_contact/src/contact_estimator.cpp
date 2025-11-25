#include "bumpybot_torque_contact/contact_estimator.hpp"
#include <pluginlib/class_list_macros.h>
#include <cmath>

namespace bumpybot_torque_contact {

ContactEstimator::ContactEstimator()
  : nh_()
  , private_nh_("~")
  , com_x_(0.0)
  , com_y_(0.0)
  , last_cp_x_(0.0)
  , last_cp_y_(0.0)
  , visualize_threshold_(0.025)
  , R_(0.248195487469)
  , wheel_radius_(0.1)
  , roller_radius_(0.00918135)
  , mass_(80.0)
  , Br_alpha_(0.2)
  , Br_beta_(0.4)
  , F_MIN_(0.5)
  , scale_(1.0)
  , Ib_(0.0)
  , PAR_TOL_(1e-8)
  , has_theta_(false)
  , has_acc_(false)
  , has_ang_accel_(false)
  , has_torque_(false)
  , theta_(0.0)
  , wz_(0.0)
  , last_wz_time_(0.0)
  , angular_accel_z_(0.0)
  , vx_(0.0)
  , vy_(0.0)
  , enable_outlier_rejection_(true)
  , enable_outlier_reset_(false)
  , outlier_threshold_ratio_(2.0)
  , outlier_window_size_(5)
  , last_valid_cx_(0.0)
  , last_valid_cy_(0.0)
  , last_valid_Fx_(0.0)
  , last_valid_Fy_(0.0)
{
  acc_.setZero();
  torque_sensed_.setZero();
  M_mat_.setZero();
  Jcw_.setZero();
  Jcwinv_.setZero();
  Jcr_.setZero();
}

void ContactEstimator::onInit()
{
  nh_        = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Load physical params (same defaults as Python)
  private_nh_.param("R",                 R_,              R_);
  private_nh_.param("wheel_radius",      wheel_radius_,   wheel_radius_);
  private_nh_.param("roller_radius",     roller_radius_,  roller_radius_);
  private_nh_.param("mass",              mass_,           mass_);
  private_nh_.param("roller_damping_alpha", Br_alpha_,    Br_alpha_);
  private_nh_.param("roller_damping_beta",  Br_beta_,     Br_beta_);
  private_nh_.param("F_MIN",             F_MIN_,          F_MIN_);
  private_nh_.param("scale",             scale_,          scale_);
  private_nh_.param("visualize_threshold", visualize_threshold_, visualize_threshold_);
  private_nh_.param("parallel_tolerance", PAR_TOL_,       PAR_TOL_);

  // Build mass / inertia matrices
  const double L = std::sqrt(3.0) * R_;
  M_mat_.setZero();
  M_mat_(0,0) = mass_;
  M_mat_(1,1) = mass_;
  M_mat_(2,2) = L*L / 2.0;
  Ib_ = 0.5 * mass_ * L*L;

  // Load outline vertices from param and setup marker & COM
  if (!loadOutlineParam()) {
    NODELET_FATAL("Failed to load BUMPYBOT_OUTLINE; ContactEstimator will not run.");
    throw std::runtime_error("ContactEstimator initialization failed: outline param missing");
  }

  computeCOM();
  last_cp_x_ = com_x_;
  last_cp_y_ = com_y_;

  setupOutlineMarker();
  setupForceMarkers();

  // Publishers
  outline_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "bumpybot_outline_marker", 1, true);

  contact_sphere_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "contact_point", 1);

  force_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "force_arrow", 1);

  force_values_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "external_force_values", 1);

  outline_marker_pub_.publish(outline_marker_);

  // Dynamic reconfigure
  dr_server_.reset(new dynamic_reconfigure::Server<JacobianConfig>(private_nh_));
  dynamic_reconfigure::Server<JacobianConfig>::CallbackType cb =
      boost::bind(&ContactEstimator::dynConfigCallback, this, _1, _2);
  dr_server_->setCallback(cb);

  // Message_filters subscribers and sync
  imu_sub_.subscribe(nh_, "/imu/data", 10);
  odom_sub_.subscribe(nh_, "/odometry/filtered", 10);
  torque_sub_.subscribe(nh_, "/filtered_torque_data", 10);
  wheel_sub_.subscribe(nh_, "/joint_states", 10);

  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(3), imu_sub_, odom_sub_, torque_sub_, wheel_sub_));
  sync_->registerCallback(
      boost::bind(&ContactEstimator::syncedCallback, this, _1, _2, _3, _4));

  // Health monitoring subscribers
  health_imu_sub_ = nh_.subscribe("/imu/data", 10,
                                  &ContactEstimator::healthImuCb, this);
  health_odom_sub_ = nh_.subscribe("/odometry/filtered", 10,
                                   &ContactEstimator::healthOdomCb, this);
  health_torque_sub_ = nh_.subscribe("/filtered_torque_data", 10,
                                     &ContactEstimator::healthTorqueCb, this);
  health_wheel_sub_ = nh_.subscribe("/joint_states", 10,
                                    &ContactEstimator::healthWheelCb, this);

                                    
  health_timer_ = nh_.createTimer(ros::Duration(0.5),
                                  &ContactEstimator::checkTopicHealth,
                                  this);

  // Wait for /clock like Python did (in sim)
  while (!ros::Time::waitForValid(ros::WallDuration(0.1))) {
    NODELET_INFO_THROTTLE(2.0, "ContactEstimator: Waiting for /clock to start...");
  }

  NODELET_INFO("ContactEstimator nodelet initialized.");
}

//------------------------------------------------------------------------------
// Outline loading & COM
//------------------------------------------------------------------------------

bool ContactEstimator::loadOutlineParam()
{
  ROS_INFO("Loading ContactEstimator outline from BUMPYBOT_OUTLINE...");
  XmlRpc::XmlRpcValue outline;
  if (!nh_.getParam("BUMPYBOT_OUTLINE", outline)) {
    NODELET_ERROR("Param BUMPYBOT_OUTLINE not found. Did you load the YAML?");
    return false;
  }

  if (outline.getType() != XmlRpc::XmlRpcValue::TypeArray || outline.size() == 0) {
    NODELET_ERROR("BUMPYBOT_OUTLINE must be a non-empty YAML list.");
    return false;
  }

  XmlRpc::XmlRpcValue first = outline[0];
  if (first.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !first.hasMember("vertices")) {
    NODELET_ERROR("BUMPYBOT_OUTLINE[0] must be a map with key 'vertices'.");
    return false;
  }

  XmlRpc::XmlRpcValue &verts = first["vertices"];
  if (verts.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    NODELET_ERROR("BUMPYBOT_OUTLINE[0].vertices must be a list.");
    return false;
  }

  outline_vertices_.clear();
  outline_vertices_.reserve(verts.size());

  for (int i = 0; i < verts.size(); ++i) {
    XmlRpc::XmlRpcValue &v = verts[i];

    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() != 2) {
      NODELET_ERROR("BUMPYBOT_OUTLINE[0].vertices[%d] is not a 2-element list.", i);
      return false;
    }

    if (v[0].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        v[0].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      NODELET_ERROR("Vertex %d x is not numeric.", i);
      return false;
    }
    if (v[1].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        v[1].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      NODELET_ERROR("Vertex %d y is not numeric.", i);
      return false;
    }

    geometry_msgs::Point p;
    p.x = static_cast<double>(v[0]);
    p.y = static_cast<double>(v[1]);
    p.z = 0.0;

    outline_vertices_.push_back(p);
  }

  NODELET_INFO("Loaded %zu vertices from BUMPYBOT_OUTLINE.",
               outline_vertices_.size());
  return true;
}

void ContactEstimator::computeCOM()
{
  if (outline_vertices_.empty()) {
    com_x_ = 0.0;
    com_y_ = 0.0;
    NODELET_WARN("Outline vertices empty; COM set to (0,0).");
    return;
  }

  double sx = 0.0;
  double sy = 0.0;
  for (const auto &p : outline_vertices_) {
    sx += p.x;
    sy += p.y;
  }
  const double N = static_cast<double>(outline_vertices_.size());
  com_x_ = sx / N;
  com_y_ = sy / N;

  NODELET_INFO("Computed COM from outline: (%.6f, %.6f)", com_x_, com_y_);
}

//------------------------------------------------------------------------------
// Visualization setup & timer
//------------------------------------------------------------------------------

void ContactEstimator::setupOutlineMarker()
{
  outline_marker_.header.frame_id = "bumpybot_outline";
  outline_marker_.ns = "bumpybot_outline";
  outline_marker_.id = 0;
  outline_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  outline_marker_.action = visualization_msgs::Marker::ADD;

  outline_marker_.scale.x = 0.01;  // line width

  outline_marker_.color.r = 0.0;
  outline_marker_.color.g = 1.0;
  outline_marker_.color.b = 0.0;
  outline_marker_.color.a = 1.0;

  outline_marker_.frame_locked = true;           // helps with moving frames
  outline_marker_.header.stamp = ros::Time(0);  
  outline_marker_.points.clear();
  outline_marker_.points.reserve(outline_vertices_.size() + 1);

  for (const auto &v : outline_vertices_) {
    outline_marker_.points.push_back(v);
  }

  if (!outline_marker_.points.empty()) {
    outline_marker_.points.push_back(outline_marker_.points.front());
  }
}

void ContactEstimator::setupForceMarkers()
{
  sphere_marker_.header.frame_id = "base_link";
  sphere_marker_.ns = "contact_point";
  sphere_marker_.id = 0;
  sphere_marker_.type = visualization_msgs::Marker::SPHERE;
  sphere_marker_.scale.x = 0.025;
  sphere_marker_.scale.y = 0.025;
  sphere_marker_.scale.z = 0.025;
  sphere_marker_.color.r = 1.0;
  sphere_marker_.color.g = 0.0;
  sphere_marker_.color.b = 0.0;
  sphere_marker_.color.a = 1.0;

  arrow_marker_.header.frame_id = "base_link";
  arrow_marker_.ns = "force_arrow";
  arrow_marker_.id = 0;
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  arrow_marker_.scale.x = 0.1;
  arrow_marker_.scale.y = 0.1;
  arrow_marker_.scale.z = 0.1;
  arrow_marker_.color.b = 1.0;
  arrow_marker_.color.a = 1.0;
}

void ContactEstimator::timerCallback(const ros::TimerEvent &)
{
  // Periodically publish the outline marker
  outline_marker_.header.stamp = ros::Time::now();
  outline_marker_pub_.publish(outline_marker_);
}

//------------------------------------------------------------------------------
// Dynamic reconfigure
//------------------------------------------------------------------------------

void ContactEstimator::dynConfigCallback(JacobianConfig &config, uint32_t)
{
  mass_     = config.mass;
  R_        = config.R;
  Br_alpha_ = config.roller_damping_alpha;
  Br_beta_  = config.roller_damping_beta;
  scale_    = config.scale;
  enable_outlier_rejection_ = config.enable_outlier_rejection;
  outlier_threshold_ratio_  = config.outlier_threshold_ratio;
  outlier_window_size_      = config.outlier_window_size;
  enable_outlier_reset_     = config.enable_outlier_reset;
  F_MIN_                    = config.F_MIN;

  // Recompute inertia terms if mass or R changed
  const double L = std::sqrt(3.0) * R_;
  M_mat_.setZero();
  M_mat_(0,0) = mass_;
  M_mat_(1,1) = mass_;
  M_mat_(2,2) = L*L / 2.0;
  Ib_         = 0.5 * mass_ * L*L;

  // Rebuild force magnitude buffer if window size changed
  if (static_cast<int>(force_mag_buffer_.max_size()) != outlier_window_size_) {
    force_mag_buffer_.clear();
    force_mag_buffer_ = std::deque<double>();
  }

  NODELET_INFO("ContactEstimator reconfigure: mass=%.3f R=%.3f F_MIN=%.3f",
               mass_, R_, F_MIN_);
}

//------------------------------------------------------------------------------
// Synced callback
//------------------------------------------------------------------------------

void ContactEstimator::syncedCallback(
    const sensor_msgs::ImuConstPtr       &imu_msg,
    const nav_msgs::OdometryConstPtr     &odom_msg,
    const sensor_msgs::JointStateConstPtr &torque_msg,
    const sensor_msgs::JointStateConstPtr &wheel_msg)
{
  const ros::Time now = ros::Time::now();
  last_imu_msg_time_    = now;
  last_odom_msg_time_   = now;
  last_torque_msg_time_ = now;
  last_wheel_msg_time_  = now;

  // IMU: linear acceleration
  const double ax = imu_msg->linear_acceleration.x;
  const double ay = imu_msg->linear_acceleration.y;
  acc_ << ax, ay, 0.0;
  has_acc_ = true;

  // ODOM: yaw and ang vel + ang accel
  const geometry_msgs::Quaternion &q = odom_msg->pose.pose.orientation;
  tf::Quaternion tfq(q.x, q.y, q.z, q.w);
  theta_ = tf::getYaw(tfq);
  has_theta_ = true;

  vx_ = 0.0;  // odom_msg->twist.twist.linear.x;
  vy_ = 0.0;  // odom_msg->twist.twist.linear.y;

  const double wz = odom_msg->twist.twist.angular.z;
  const double tstamp = odom_msg->header.stamp.toSec();

  if (!std::isfinite(last_wz_time_)) {
    angular_accel_z_ = 0.0;
  } else {
    const double dt = tstamp - last_wz_time_;
    if (dt > 1e-6) {
      angular_accel_z_ = (wz - wz_) / dt;
    } else {
      angular_accel_z_ = 0.0;
    }
  }
  wz_ = wz;
  last_wz_time_ = tstamp;
  has_ang_accel_ = true;

  // Torque: assume 3 elements
  if (torque_msg->position.size() < 3) {
    NODELET_WARN_THROTTLE(1.0, "Torque message has < 3 positions; skipping.");
    return;
  }
  torque_sensed_ << torque_msg->position[0],
                     torque_msg->position[1],
                     torque_msg->position[2];
  has_torque_ = true;

  // Check readiness
  if (!has_theta_ || !has_acc_ || !has_ang_accel_ || !has_torque_)
    return;

  buildJacobians(theta_);

  double cx, cy, Fx, Fy;
  bool   ok;
  if (!externalForces(cx, cy, Fx, Fy, ok)) {
    return;
  }

  double fcx, fcy, fFx, fFy;
  bool is_outlier = false;
  applyOutlierFilter(cx, cy, Fx, Fy, fcx, fcy, fFx, fFy, is_outlier);

  const bool final_ok = ok && !is_outlier;

  visualize(fcx, fcy, fFx, fFy, final_ok);

  std_msgs::Float64MultiArray out;
  out.data.resize(5);
  out.data[0] = fcx;
  out.data[1] = fcy;
  out.data[2] = fFx;
  out.data[3] = fFy;
  out.data[4] = final_ok ? 1.0 : 0.0;
  force_values_pub_.publish(out);
}

//------------------------------------------------------------------------------
// Jacobians & external forces
//------------------------------------------------------------------------------

void ContactEstimator::buildJacobians(double th)
{
  const double rw = wheel_radius_;
  const double rr = roller_radius_;

  Jcw_ <<
    -std::sin(th),                std::cos(th),                R_,
    -std::sin(th + 2.0*M_PI/3.0), std::cos(th + 2.0*M_PI/3.0), R_,
    -std::sin(th + 4.0*M_PI/3.0), std::cos(th + 4.0*M_PI/3.0), R_;
  Jcw_ *= (1.0 / rw);

  Jcwinv_ = Jcw_.inverse();

  Jcr_ <<
    std::cos(th),                std::sin(th),                0.0,
    std::cos(th + 2.0*M_PI/3.0), std::sin(th + 2.0*M_PI/3.0), 0.0,
    std::cos(th + 4.0*M_PI/3.0), std::sin(th + 4.0*M_PI/3.0), 0.0;
  Jcr_ *= (1.0 / rr);
}

bool ContactEstimator::externalForces(double &cx, double &cy,
                                      double &Fx, double &Fy,
                                      bool &ok)
{
  // Roller damping: Br_k = alpha * tanh(beta * qr_dot)
  Eigen::Vector3d Xd;
  Xd << vx_, vy_, wz_;
  Eigen::Vector3d qr_dot = Jcr_ * Xd;

  Eigen::Vector3d Br_vec;
  Br_vec = Br_alpha_ * (Br_beta_ * qr_dot).array().tanh().matrix();

  // Inertia wrench term: M * X_dd + J_cr^T * B_r
  Eigen::Vector3d X_dd;
  X_dd << acc_(0), acc_(1), angular_accel_z_;

  Eigen::Vector3d T_noF =
      Jcwinv_.transpose() * (M_mat_ * X_dd + Jcr_.transpose() * Br_vec);

  // Residual torque and body-frame wrench
  Eigen::Vector3d deltaT = T_noF - torque_sensed_;
  Eigen::Vector3d W      = scale_ * (Jcw_.transpose() * deltaT);

  Fx = W(0);
  Fy = W(1);
  const double Mz = W(2);

  const double mag = std::hypot(Fx, Fy);
  if (mag < F_MIN_) {
    // Below force gate: keep last contact point, zero forces, mark not-ok
    cx = last_cp_x_;
    cy = last_cp_y_;
    Fx = 0.0;
    Fy = 0.0;
    ok = false;
    return true;
  }

  bool good = false;
  if (!forceLineIntersection(Fx, Fy, Mz, cx, cy, good)) {
    ok = false;
    return false;  // serious geometric failure
  }

  ok = good;
  return true;
}


bool ContactEstimator::forceLineIntersection(double Fx, double Fy, double Mz,
                                             double &cx, double &cy,
                                             bool &ok)
{
  const size_t N = outline_vertices_.size();
  if (N == 0) {
    cx = last_cp_x_;
    cy = last_cp_y_;
    ok = false;
    return true;  // nothing to intersect with
  }

  // Force direction and magnitude
  const double mag = std::hypot(Fx, Fy);
  if (mag <= 0.0) {
    cx = last_cp_x_;
    cy = last_cp_y_;
    ok = false;
    return true;
  }

  Eigen::Vector2d u(Fx / mag, Fy / mag);      // unit force direction
  Eigen::Vector2d u_perp(-u.y(), u.x());      // +90° rotated direction
  Eigen::Vector2d C0(com_x_, com_y_);

  // Residual line-of-action offset: r_perp = Mz / ||F||
  const double r_perp = Mz / mag;

  // Starting point of ray, possibly offset from centroid
  Eigen::Vector2d p0 = C0 + r_perp * u_perp;

  const double eps_angle = PAR_TOL_;  // used as |det(u, d_i)| threshold

  double best_s  = std::numeric_limits<double>::infinity();
  bool   found   = false;

  for (size_t i = 0; i < N; ++i) {
    size_t j = (i + 1) % N;

    Eigen::Vector2d v_i(outline_vertices_[i].x, outline_vertices_[i].y);
    Eigen::Vector2d v_j(outline_vertices_[j].x, outline_vertices_[j].y);
    Eigen::Vector2d d  = v_j - v_i;                   // edge vector

    // det(u, d) = u_x d_y - u_y d_x
    const double denom = u.x() * d.y() - u.y() * d.x();
    if (std::fabs(denom) < eps_angle) {
      // Edge almost parallel to ray: skip
      continue;
    }

    // w = v_i - p0
    Eigen::Vector2d w = v_i - p0;

    // s_i = det(w, d) / det(u, d)
    // t_i = det(w, u) / det(u, d)
    const double s = (w.x() * d.y() - w.y() * d.x()) / denom;
    const double t = (w.x() * u.y() - w.y() * u.x()) / denom;

    // Require intersection in front of ray origin and on segment
    if (s < 0.0 || t < 0.0 || t > 1.0) {
      continue;
    }

    // Optional pushing vs pulling test using inward normal:
    // For CCW polygon, inward normal is n_in = R(+90°) d / ||d||
    Eigen::Vector2d n_in(-d.y(), d.x());
    const double n_norm = n_in.norm();
    if (n_norm > 1e-9) {
      n_in /= n_norm;
      if (n_in.dot(u) <= 0.0) {
        // Force not pushing into polygon; skip this edge
        continue;
      }
    }

    if (s < best_s) {
      best_s = s;
      found  = true;
    }
  }

  if (!found || !std::isfinite(best_s)) {
    // fall back to last valid contact point
    cx = last_cp_x_;
    cy = last_cp_y_;
    ok = false;
    return true;
  }

  // First intersection along the ray
  Eigen::Vector2d cp = p0 + best_s * u;
  cx = cp.x();
  cy = cp.y();

  last_cp_x_ = cx;
  last_cp_y_ = cy;
  ok = true;
  return true;
}


//------------------------------------------------------------------------------
// Outlier filter
//------------------------------------------------------------------------------

void ContactEstimator::applyOutlierFilter(double cx, double cy,
                                          double Fx, double Fy,
                                          double &fcx, double &fcy,
                                          double &fFx, double &fFy,
                                          bool &is_outlier)
{
  const double mag = std::hypot(Fx, Fy);
  is_outlier = false;

  if (!enable_outlier_rejection_) {
    // Always accept
    if (mag > 0.0) {
      force_mag_buffer_.push_back(mag);
    }
    last_valid_cx_ = cx;
    last_valid_cy_ = cy;
    last_valid_Fx_ = Fx;
    last_valid_Fy_ = Fy;
    fcx = cx; fcy = cy; fFx = Fx; fFy = Fy;
    return;
  }

  if (mag > 0.0) {
    force_mag_buffer_.push_back(mag);
    // trim if larger than window
    while (static_cast<int>(force_mag_buffer_.size()) > outlier_window_size_) {
      force_mag_buffer_.pop_front();
    }
  }

  double median_mag = 0.0;
  if (!force_mag_buffer_.empty()) {
    std::vector<double> buf(force_mag_buffer_.begin(), force_mag_buffer_.end());
    std::sort(buf.begin(), buf.end());
    const size_t mid = buf.size() / 2;
    if (buf.size() % 2 == 1) {
      median_mag = buf[mid];
    } else {
      median_mag = 0.5 * (buf[mid-1] + buf[mid]);
    }
  }

  const double threshold = outlier_threshold_ratio_ * (median_mag + 1e-9);
  if (mag > threshold) {
    is_outlier = true;
  }

  if (!is_outlier) {
    last_valid_cx_ = cx;
    last_valid_cy_ = cy;
    last_valid_Fx_ = Fx;
    last_valid_Fy_ = Fy;
    fcx = cx; fcy = cy; fFx = Fx; fFy = Fy;
    return;
  }

  // Outlier case
  if (enable_outlier_reset_) {
    fcx = last_valid_cx_;
    fcy = last_valid_cy_;
    fFx = last_valid_Fx_;
    fFy = last_valid_Fy_;
  } else {
    fcx = cx; fcy = cy; fFx = Fx; fFy = Fy;
  }
}

//------------------------------------------------------------------------------
// Visualization of contact point & force
//------------------------------------------------------------------------------

void ContactEstimator::visualize(double cx, double cy,
                                 double Fx, double Fy,
                                 bool hit)
{
  const double norm = std::hypot(Fx, Fy);
  if (!hit) {
    ROS_WARN_THROTTLE(1.0, "No valid intersection to visualize");
    return;
  }

  // Sphere at contact point
  sphere_marker_.header.stamp = ros::Time::now();
  sphere_marker_.pose.position.x = cx;
  sphere_marker_.pose.position.y = cy;
  sphere_marker_.pose.position.z = 0.0;
  contact_sphere_pub_.publish(sphere_marker_);

  if (norm < visualize_threshold_) {
    arrow_marker_.points.clear();
    arrow_marker_.header.stamp = ros::Time::now();
    force_arrow_pub_.publish(arrow_marker_);
    return;
  }

  geometry_msgs::Point start;
  geometry_msgs::Point end;

  start.x = cx;
  start.y = cy;
  start.z = 0.0;

  end.x = cx + 0.5 * Fx / norm;
  end.y = cy + 0.5 * Fy / norm;
  end.z = 0.0;

  arrow_marker_.points.clear();
  arrow_marker_.points.push_back(start);
  arrow_marker_.points.push_back(end);
  arrow_marker_.header.stamp = ros::Time::now();
  force_arrow_pub_.publish(arrow_marker_);
}

//------------------------------------------------------------------------------
// Health monitoring
//------------------------------------------------------------------------------

void ContactEstimator::healthImuCb(const sensor_msgs::ImuConstPtr &)
{
  last_imu_msg_time_ = ros::Time::now();
}

void ContactEstimator::healthOdomCb(const nav_msgs::OdometryConstPtr &)
{
  last_odom_msg_time_ = ros::Time::now();
}

void ContactEstimator::healthTorqueCb(const sensor_msgs::JointStateConstPtr &)
{
  last_torque_msg_time_ = ros::Time::now();
}

void ContactEstimator::healthWheelCb(const sensor_msgs::JointStateConstPtr &)
{
  last_wheel_msg_time_ = ros::Time::now();
}

void ContactEstimator::checkTopicHealth(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  const double timeout = 2.0;

  if (!last_imu_msg_time_.isZero() &&
      (now - last_imu_msg_time_).toSec() > timeout) {
    ROS_WARN_THROTTLE(5.0, "Topic /imu/data has not published recently");
  }
  if (!last_odom_msg_time_.isZero() &&
      (now - last_odom_msg_time_).toSec() > timeout) {
    ROS_WARN_THROTTLE(5.0, "Topic /odometry/filtered has not published recently");
  }
  if (!last_torque_msg_time_.isZero() &&
      (now - last_torque_msg_time_).toSec() > timeout) {
    ROS_WARN_THROTTLE(5.0, "Topic /filtered_torque_data has not published recently");
  }
  if (!last_wheel_msg_time_.isZero() &&
      (now - last_wheel_msg_time_).toSec() > timeout) {
    ROS_WARN_THROTTLE(5.0, "Topic /joint_states has not published recently");
  }
}

}  // namespace bumpybot_torque_contact

// Register nodelet
PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::ContactEstimator,
                       nodelet::Nodelet)
