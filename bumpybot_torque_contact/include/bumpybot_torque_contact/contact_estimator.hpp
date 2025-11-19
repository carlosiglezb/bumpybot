#ifndef BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP
#define BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
namespace bumpybot_torque_contact {

class ContactEstimator : public nodelet::Nodelet
{
public:
  ContactEstimator();


private:
  virtual void onInit();
  bool loadParams();  

  // Example: a timer and a publisher
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher  debug_pub_;
  ros::Timer      timer_;

  ros::Publisher marker_pub_;
  std::vector<geometry_msgs::Point> outline_vertices_;  // holds the outline polygon loaded in from ros param
  visualization_msgs::Marker outline_marker_;
    void setupOutlineMarker();
  void timerCallback(const ros::TimerEvent &event);
};

}  // namespace bumpybot_torque_contact

#endif  // BUMPYBOT_TORQUE_CONTACT_CONTACT_ESTIMATOR_HPP
