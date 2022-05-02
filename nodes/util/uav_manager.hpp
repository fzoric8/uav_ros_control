#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/service_callback_helper.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <uav_ros_msgs/ArmAndTakeoff.h>
#include <uav_ros_msgs/Land.h>
#include <uav_ros_lib/topic_handler.hpp>
#include <mutex>

namespace uav_ros_control {

class UAVManager : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  static constexpr auto  PRE_TAKEOFF_STATUS       = "CARROT_ON_LAND";
  const std::vector<int> CARROT_ACTIVATION_VECTOR = { 0, 0, 0, 0, 0, 1 };
  static constexpr auto  OFFBOARD_MODE            = "GUIDED_NOGPS";
  static constexpr auto  TRACKER_ACTIVE           = "ACTIVE";
  static constexpr auto  POSITION_HOLD            = "HOLD";
  static constexpr auto  ODOM_TIMEOUT             = 0.1;
  static constexpr auto  WAIT_DURATION            = 0.1;

  // Takeoff server
  ros::ServiceServer m_armtakeoff_server;
  std::mutex         m_armtakeoff_mutex;
  bool               arm_and_takeoff_cb(uav_ros_msgs::ArmAndTakeoff::Request&  req,
                                        uav_ros_msgs::ArmAndTakeoff::Response& resp);

  // Land server
  ros::ServiceServer m_land_server;
  std::mutex         m_land_mutex;
  bool land_cb(uav_ros_msgs::Land::Request& req, uav_ros_msgs::Land::Response& resp);

  // Subscriber handlers
  ros_util::TopicHandlerMutexed<nav_msgs::Odometry>::Ptr m_odom_handler;
  ros_util::TopicHandlerMutexed<std_msgs::String>::Ptr   m_carrot_state_handler;
  ros_util::TopicHandlerMutexed<mavros_msgs::State>::Ptr m_mavros_state_handler;
  ros_util::TopicHandlerMutexed<std_msgs::String>::Ptr   m_tracker_state_handler;

  // Service clients
  ros::ServiceClient m_takeoff_client;
  ros::ServiceClient m_land_client;
  ros::ServiceClient m_arming_client;
  ros::ServiceClient m_mode_client;
  ros::ServiceClient m_tracker_reset_client;
  ros::ServiceClient m_clear_mission;

  // Publishers
  ros::Publisher m_joy_publisher;
};

}// namespace uav_ros_control
