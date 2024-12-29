
// Based on IRA Laser Tools package's laserscan_multi_merger.cpp
// see  https://github.com/iralabdisco/ira_laser_tools.

//This node merges multiple incoming LaserScan topics into a single point cloud (published at /merged_cloud) and also converts that merged cloud back into a single LaserScan (published at /scan_multi). 
#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>
#include <mutex>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic);
    void publishMergedCloud(const ros::TimerEvent &);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);
    
private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    std::mutex data_mutex_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> scan_subscribers;
    ros::Timer publish_timer_;

    vector<pcl::PCLPointCloud2> clouds;
    vector<ros::Time> cloud_timestamps;
    vector<string> input_topics;

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    double publish_frequency; // New parameter for publish rate

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
    this->angle_min = config.angle_min;
    this->angle_max = config.angle_max;
    this->angle_increment = config.angle_increment;
    this->time_increment = config.time_increment;
    this->scan_time = config.scan_time;
    this->range_min = config.range_min;
    this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
    // LaserScan topics to subscribe
    ros::master::V_TopicInfo topics;

    istringstream iss(laserscan_topics);
    set<string> tokens;
    copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
    vector<string> tmp_input_topics;

    while (!tokens.empty())
    {
        ROS_INFO("Waiting for topics ...");
        ros::master::getTopics(topics);
        ros::Duration(1.0).sleep();

        for (size_t i = 0; i < topics.size(); i++)
        {
            if (topics[i].datatype == "sensor_msgs/LaserScan" && tokens.erase(topics[i].name) > 0)
            {
                tmp_input_topics.push_back(topics[i].name);
            }
        }
    }

    sort(tmp_input_topics.begin(), tmp_input_topics.end());
    std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(last, tmp_input_topics.end());

    // Do not re-subscribe if the topics are the same
    if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
    {

        // Unsubscribe from previous topics
        for (size_t i = 0; i < scan_subscribers.size(); i++)
            scan_subscribers[i].shutdown();

        input_topics = tmp_input_topics;

        if (input_topics.size() > 0)
        {
            scan_subscribers.resize(input_topics.size());
            clouds.resize(input_topics.size());
            cloud_timestamps.resize(input_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
            for (size_t i = 0; i < input_topics.size(); ++i)
            {
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>(input_topics[i], 10, boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topics[i]));
                cloud_timestamps[i] = ros::Time(0);
                cout << input_topics[i] << " ";
            }
            cout << endl;
        }
        else
            ROS_INFO("Not subscribed to any topic.");
    }
}

LaserscanMerger::LaserscanMerger()
{
    ros::NodeHandle nh("~");

    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
    nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    nh.param<std::string>("laserscan_topics", laserscan_topics, "");
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);
    nh.param("publish_frequency", publish_frequency, 10.0); // Default to 10 Hz

    this->laserscan_topic_parser();

    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
    laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);

    // Initialize timer for merging and publishing
    publish_timer_ = node_.createTimer(ros::Duration(1.0 / publish_frequency), &LaserscanMerger::publishMergedCloud, this);
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
    sensor_msgs::PointCloud2 tmpCloud3;

    try
    {
        // Transform the laser scan into a point cloud in the target frame
        tfListener_.waitForTransform(scan->header.frame_id, destination_frame, scan->header.stamp, ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
        tfListener_.transformPointCloud(destination_frame, tmpCloud1, tmpCloud2);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Transform exception for topic %s: %s", topic.c_str(), ex.what());
        // Clear the cloud data and reset timestamp
        for (size_t i = 0; i < input_topics.size(); i++)
        {
            if (topic.compare(input_topics[i]) == 0)
            {
                clouds[i].data.clear();
                cloud_timestamps[i] = ros::Time(0);
            }
        }
        return;
    }

    // Update the corresponding cloud and timestamp
    for (size_t i = 0; i < input_topics.size(); i++)
    {
        if (topic.compare(input_topics[i]) == 0)
        {
            sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
            pcl_conversions::toPCL(tmpCloud3, clouds[i]);
            cloud_timestamps[i] = scan->header.stamp;
        }
    }
}

void LaserscanMerger::publishMergedCloud(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    pcl::PCLPointCloud2 merged_cloud;
    bool first_cloud = true;
    ros::Time current_time = ros::Time::now();
    ros::Duration timeout(1.0); // Adjust as needed
    int valid_clouds = 0;
    for (size_t i = 0; i < clouds.size(); i++)
    {
        if (clouds[i].data.empty())
            continue;

        // Check if the cloud is recent
        if ((current_time - cloud_timestamps[i]) > timeout)
        {
            ROS_WARN_THROTTLE(5, "Cloud from topic %s is stale. Clearing data.", input_topics[i].c_str());
            clouds[i].data.clear(); // Clear stale data
            cloud_timestamps[i] = ros::Time(0); // Reset timestamp
            continue;
        }


        if (first_cloud)
        {
            merged_cloud = clouds[i];
            first_cloud = false;
        }
        else
        {
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
            pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
#else
            pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif
        }
    valid_clouds++;
    }
    // ROS_INFO("Merging %d valid clouds", valid_clouds);
    if (!first_cloud)
    {
        // Publish the merged point cloud
        sensor_msgs::PointCloud2 output_cloud;
        pcl_conversions::fromPCL(merged_cloud, output_cloud);
        // ROS_INFO("Merged cloud has %d points", output_cloud.width * output_cloud.height);
        output_cloud.header.frame_id = destination_frame;
        output_cloud.header.stamp = current_time;

        point_cloud_publisher_.publish(output_cloud);

        // Convert merged cloud to Eigen matrix for laser scan conversion
        Eigen::MatrixXf points;
        getPointCloudAsEigen(merged_cloud, points);

        pointcloud_to_laserscan(points, &merged_cloud);
    }
    else
    {
        ROS_WARN_THROTTLE(5, "No valid clouds to merge. Skipping publish.");
    }
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = pcl_conversions::fromPCL(merged_cloud->header);
    output->header.stamp = ros::Time::now(); // Update timestamp
    output->angle_min = this->angle_min;
    output->angle_max = this->angle_max;
    output->angle_increment = this->angle_increment;
    output->time_increment = this->time_increment;
    output->scan_time = this->scan_time;
    output->range_min = this->range_min;
    output->range_max = this->range_max;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for (int i = 0; i < points.cols(); i++)
    {
        const float &x = points(0, i);
        const float &y = points(1, i);
        const float &z = points(2, i);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = pow(y, 2) + pow(x, 2);
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_)
        {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
            output->ranges[index] = sqrt(range_sq);
    }

    laser_scan_publisher_.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback, &_laser_merger, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
