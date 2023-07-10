//
// Created by Carlos on 10/29/21.
//

#ifndef BumpybotHWInterface_H
#define BumpybotHWInterface_H

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ROS Control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// SOEM
#include <ethercat.h>

// URDF
#include <urdf/model.h>

// Messages
#include <std_msgs/Float64.h>
//#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <bumpybot_hw_interface/ServoTemp.h>

// Services
#include <bumpybot_hw_interface/ClearFaults.h>

// Boost
#include <boost/thread.hpp>

namespace bumpybot_hw
{
  class BumpybotHWInterface : public hardware_interface::RobotHW
  {
  public:
      BumpybotHWInterface(ros::NodeHandle &nh, urdf::Model* urdf_model = NULL);
      ~BumpybotHWInterface();

      bool init();
      void read(const ros::Time& time, const ros::Duration& period);
      void write(const ros::Time& time, const ros::Duration& period);

      void cmd1Callback(const std_msgs::Float64& desired_torque);
      void cmd2Callback(const std_msgs::Float64& desired_torque);
      void cmd3Callback(const std_msgs::Float64& desired_torque);

      bool initializeEthercat();
      void generateAllSlaveMappings();
      void startOperation();
      bool stopOperation();
      bool createEtherCATCheckThread();
      void EtherCATCheck();
      void EtherCATLoop();
      void EtherCATTimerThread();
      void EtherCATLoop(const ros::TimerEvent&);
      bool clearFaults(bumpybot_hw_interface::ClearFaults::Request &req,
				bumpybot_hw_interface::ClearFaults::Response &res);

      void loadURDF(ros::NodeHandle &nh, std::string param_name);

  protected:

      ros::NodeHandle nh_;
      std::string name_;

      // ROS publishers/subscribers
      ros::Publisher js_pub;
      ros::Publisher temp_pub;
      ros::Subscriber js1_sub;
      ros::Subscriber js2_sub;
      ros::Subscriber js3_sub;

      // ROS services
      ros::ServiceServer clear_faults_srv;

      // ROS messages published
      sensor_msgs::JointState js_msg;     // wheel position, velocity
      sensor_msgs::JointState temp_msg;  // array with servos temperatures
//      bumpybot_hw_interface::ServoTemp temp_msg;  // array with servos temperatures
//      std_msgs::Float64 des_torque1_msg;   // desired wheel 1 torque
//      std_msgs::Float64 des_torque2_msg;   // desired wheel 2 torque

      // Hardware Interfaces
      hardware_interface::JointStateInterface joint_state_interface_;
      hardware_interface::EffortJointInterface effort_joint_interface_;

      // States
      std::vector<double> joint_position_;
      std::vector<double> joint_velocity_;
      std::vector<double> joint_effort_;
      union convIntToFloat
      {
        int32 i32;
        float f32;
      };
      union convUIntToFloat
      {
        uint32 u32;
        float f32;
      };

      // Commands
      std::vector<double> joint_effort_cmd_;

      // Robot model configuration
      std::vector<std::string> joint_names_;
      std::size_t num_joints_;
      urdf::Model* urdf_model_;

      // EtherCAT
      int m_num_slaves_ = 0;
      ros::Timer EtherCATTimer_;
      ros::CallbackQueue EtherCATUpdateQueue_;
      boost::thread EtherCATTimerThread_;
      boost::thread EtherCATCheckThread_;
      boost::recursive_mutex r_mutex_;

      std::string ifname_;
      char IOmap[4096];
      int expectedWKC;
      boolean needlf;
      volatile int wkc;
      boolean inOP;
      uint8 currentgroup = 0;
      std::vector<double> wheel_encoder_counts_;
      std::vector<double> wheel_encoder_sign_;

  };
} // namespace

#endif //BumpybotHWInterface_H
