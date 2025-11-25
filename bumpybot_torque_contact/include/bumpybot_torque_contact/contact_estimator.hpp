#ifndef BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP
#define BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP

#include <vector>
#include <deque>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <bumpybot_torque_contact/JacobianConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace bumpybot_torque_contact {

class ContactEstimator : public nodelet::Nodelet
{
public:
  ContactEstimator();

private:
  // Nodelet
  virtual void onInit();

  // Parameter loading
  bool loadOutlineParam();
  void computeCOM();

  // Visualization
  void setupOutlineMarker();
  void setupForceMarkers();
  void timerCallback(const ros::TimerEvent &event);

  // Dynamic reconfigure
  void dynConfigCallback(JacobianConfig &config, uint32_t level);

  // Data path
  void syncedCallback(const sensor_msgs::ImuConstPtr       &imu_msg,
                      const nav_msgs::OdometryConstPtr     &odom_msg,
                      const sensor_msgs::JointStateConstPtr &torque_msg,
                      const sensor_msgs::JointStateConstPtr &wheel_msg);

  void buildJacobians(double theta);
  bool externalForces(double &cx, double &cy, double &Fx, double &Fy, bool &ok);
  bool forceLineIntersection(double Fx, double Fy, double Mz,
                            double &cx, double &cy,
                            bool &ok);


  void applyOutlierFilter(double cx, double cy, double Fx, double Fy,
                          double &fcx, double &fcy,
                          double &fFx, double &fFy,
                          bool &is_outlier);

  void visualize(double cx, double cy, double Fx, double Fy, bool hit);

  // Health monitoring
  void healthImuCb(const sensor_msgs::ImuConstPtr &msg);
  void healthOdomCb(const nav_msgs::OdometryConstPtr &msg);
  void healthTorqueCb(const sensor_msgs::JointStateConstPtr &msg);
  void healthWheelCb(const sensor_msgs::JointStateConstPtr &msg);
  void checkTopicHealth(const ros::TimerEvent &event);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Timers
  ros::Timer outline_timer_;
  ros::Timer health_timer_;

  // Publishers
  ros::Publisher outline_marker_pub_;
  ros::Publisher contact_sphere_pub_;
  ros::Publisher force_arrow_pub_;
  ros::Publisher force_values_pub_;

  // Outline / geometry
  std::vector<geometry_msgs::Point> outline_vertices_;
  visualization_msgs::Marker outline_marker_;

  // COM and last contact point
  double com_x_;
  double com_y_;
  double last_cp_x_;
  double last_cp_y_;

  // Force visualization markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  double visualize_threshold_;

  // Physical parameters
  double R_;
  double wheel_radius_;
  double roller_radius_;
  double mass_;
  double Br_alpha_;
  double Br_beta_;
  double F_MIN_;
  double scale_;

  // Computed physical matrices
  Eigen::Matrix3d Jcw_;
  Eigen::Matrix3d Jcwinv_;
  Eigen::Matrix3d Jcr_;
  Eigen::Matrix3d M_mat_;
  double Ib_;          // body inertia about z
  double PAR_TOL_;     // intersection parallel tolerance

  // State variables
  bool   has_theta_;
  bool   has_acc_;
  bool   has_ang_accel_;
  bool   has_torque_;

  double theta_;
  double wz_;
  double last_wz_time_;  // seconds

  Eigen::Vector3d acc_;          // [ax, ay, 0]
  double angular_accel_z_;       // scalar
  Eigen::Vector3d torque_sensed_; // 3x1
  double vx_;
  double vy_;

  // Outlier rejection
  bool   enable_outlier_rejection_;
  bool   enable_outlier_reset_;
  double outlier_threshold_ratio_;
  int    outlier_window_size_;
  std::deque<double> force_mag_buffer_;

  // Last valid output (for reset behavior)
  double last_valid_cx_;
  double last_valid_cy_;
  double last_valid_Fx_;
  double last_valid_Fy_;

  // Dynamic reconfigure server
  boost::shared_ptr<dynamic_reconfigure::Server<JacobianConfig>> dr_server_;

  // Message filters / sync
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Imu,
      nav_msgs::Odometry,
      sensor_msgs::JointState,
      sensor_msgs::JointState> SyncPolicy;

  message_filters::Subscriber<sensor_msgs::Imu>          imu_sub_;
  message_filters::Subscriber<nav_msgs::Odometry>        odom_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>   torque_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>   wheel_sub_;

  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Health tracking
  ros::Subscriber health_imu_sub_;
  ros::Subscriber health_odom_sub_;
  ros::Subscriber health_torque_sub_;
  ros::Subscriber health_wheel_sub_;
  ros::Time last_imu_msg_time_;
  ros::Time last_odom_msg_time_;
  ros::Time last_torque_msg_time_;
  ros::Time last_wheel_msg_time_;
};

}  // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP
