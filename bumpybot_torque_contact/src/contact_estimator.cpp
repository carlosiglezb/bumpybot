#include "bumpybot_torque_contact/contact_estimator.hpp"
#include <pluginlib/class_list_macros.h>
namespace bumpybot_torque_contact {

ContactEstimator::ContactEstimator()
  : nh_(), private_nh_("~")
{
}

void ContactEstimator::onInit()
{
  nh_        = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

debug_pub_ = nh_.advertise<std_msgs::String>("contact_estimator_debug", 1);
if (!loadParams()) {
    NODELET_FATAL("Failed to load ContactEstimator parameters, exiting...");
    throw std::runtime_error("ContactEstimator initialization failed");
}

 setupOutlineMarker(); // Visualizations should get moved to its own node eventually, this is for testing 
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("bumpybot_outline_marker", 1, true);  // For testing the outline loading
  timer_ = nh_.createTimer(ros::Duration(0.5),  
                           &ContactEstimator::timerCallback,
                           this);


  NODELET_INFO("ContactEstimator nodelet initialized");



}

bool ContactEstimator::loadParams(){
  ROS_INFO("Loading ContactEstimator parameters...");
  XmlRpc::XmlRpcValue outline;
  if (!nh_.getParam("BUMPYBOT_OUTLINE", outline)) {
    ROS_ERROR("Param BUMPYBOT_OUTLINE not found, was the YAML loaded beforehand? Check your launchfile.");
    NODELET_ERROR("Param BUMPYBOT_OUTLINE not found, was the YAML loaded beforehand? Check your launchfile.");
    return false;
  }
  // Top-level: should be a list (array) with at least 1 element
  if (outline.getType() != XmlRpc::XmlRpcValue::TypeArray || outline.size() == 0)
  {
    NODELET_ERROR("BUMPYBOT_OUTLINE must be a non-empty YAML list.");
    return false;
  }
  XmlRpc::XmlRpcValue first = outline[0];
  if (first.getType() != XmlRpc::XmlRpcValue::TypeStruct || !first.hasMember("vertices"))
  {
    NODELET_ERROR("BUMPYBOT_OUTLINE[0] must be a map with key 'vertices'.");
    return false;
  }
  XmlRpc::XmlRpcValue &verts = first["vertices"];
  if (verts.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    NODELET_ERROR("BUMPYBOT_OUTLINE[0].vertices must be a list.");
    return false;
  }
  outline_vertices_.clear();
  outline_vertices_.reserve(verts.size());

  for (int i = 0; i < verts.size(); ++i)
  {
    XmlRpc::XmlRpcValue &v = verts[i];

    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() != 2)
    {
      NODELET_ERROR("BUMPYBOT_OUTLINE[0].vertices[%d] is not a 2-element list.", i);
      return false;
    }

    if (v[0].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        v[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      NODELET_ERROR("Vertex %d x is not numeric.", i);
      return false;
    }
    if (v[1].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        v[1].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      NODELET_ERROR("Vertex %d y is not numeric.", i);
      return false;
    }

    geometry_msgs::Point p;
    p.x = static_cast<double>(v[0]);
    p.y = static_cast<double>(v[1]);
    p.z = 0.0;  // 2D outline, set z to 0

    outline_vertices_.push_back(p);
  }
  
  NODELET_INFO("Loaded %zu vertices from BUMPYBOT_OUTLINE.", outline_vertices_.size());

  return true;
}



void ContactEstimator::timerCallback(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "ContactEstimator test tick";
  debug_pub_.publish(msg);

  if (outline_marker_.points.empty()) return;

  outline_marker_.header.stamp = ros::Time::now();
  marker_pub_.publish(outline_marker_);

}

void ContactEstimator::setupOutlineMarker() // Visualizations should get moved to its own node eventually, this is for testing 
{
  outline_marker_.header.frame_id = "base_link";  
  outline_marker_.ns = "bumpybot_outline";
  outline_marker_.id = 0;
  outline_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  outline_marker_.action = visualization_msgs::Marker::ADD;

  outline_marker_.scale.x = 0.01;  
  outline_marker_.color.r = 0.0;
  outline_marker_.color.g = 1.0;
  outline_marker_.color.b = 0.0;
  outline_marker_.color.a = 1.0;

  outline_marker_.points.clear();
  outline_marker_.points.reserve(outline_vertices_.size() + 1);

  for (const auto& v : outline_vertices_) {
    geometry_msgs::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = v.z;  // preserving z (should be 0 for 2D outline)
    outline_marker_.points.push_back(p);
  }

  if (!outline_marker_.points.empty()) {
    outline_marker_.points.push_back(outline_marker_.points.front());
  }
}



}  // namespace bumpybot_torque_contact

PLUGINLIB_EXPORT_CLASS(bumpybot_torque_contact::ContactEstimator,  nodelet::Nodelet)
