#include "uav_manager.hpp"
#include "ros/duration.h"
#include "ros/time.h"
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <uav_ros_msgs/TakeOff.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <memory>


void uav_ros_control::UAVManager::onInit()
{
  auto& nh         = getMTNodeHandle();
  auto& nh_private = getMTPrivateNodeHandle();

  // Initialize subscribers
  m_carrot_state_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(nh, "carrot/status");
  m_odom_handler =
    ros_util::CreateTopicHandlerMutexed<nav_msgs::Odometry>(nh, "odometry", ODOM_TIMEOUT);
  m_mavros_state_handler =
    ros_util::CreateTopicHandlerMutexed<mavros_msgs::State>(nh, "mavros/state");
  m_tracker_state_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(nh, "tracker/status");

  // Initialize services
  m_mode_client    = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  m_arming_client  = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  m_takeoff_client = nh.serviceClient<uav_ros_msgs::TakeOff>("takeoff");
  m_land_client    = nh.serviceClient<std_srvs::SetBool>("land");
  m_tracker_reset_client = nh.serviceClient<std_srvs::Empty>("tracker/reset");
  m_clear_mission        = nh.serviceClient<std_srvs::SetBool>("clear_waypoints");

  // Initialize servers
  m_armtakeoff_server =
    nh.advertiseService("uav_manager/takeoff", &UAVManager::arm_and_takeoff_cb, this);
  m_land_server = nh.advertiseService("uav_manager/land", &UAVManager::land_cb, this);

  // Initialize publishers
  m_joy_publisher = nh.advertise<sensor_msgs::Joy>("joy", 1);

  ROS_INFO("[UavManager] Initialized");
}

bool uav_ros_control::UAVManager::arm_and_takeoff_cb(
  uav_ros_msgs::ArmAndTakeoff::Request&  req,
  uav_ros_msgs::ArmAndTakeoff::Response& resp)
{
  std::lock_guard<std::mutex> lock(m_armtakeoff_mutex);

  auto start_time = ros::Time::now();

  const auto set_response = [&](bool status, std::string message) {
    resp.message = message;
    resp.success = status;
  };

  const auto check_elapsed_time = [&](const std::string& status) {
    if ((ros::Time::now() - start_time).toSec() > req.timeout) {
      return std::make_tuple<bool, std::string>(
        false,
        "UAVManager::arm_and_takeoff_cb - timeout duration elapsed while " + status);
    }
    return std::make_tuple<bool, std::string>(true, "All is well.");
  };

  // Check for set_mode service
  if (!m_mode_client.exists()) {
    set_response(false, m_mode_client.getService() + " does not exist!");
    return true;
  }

  // Check for arming service
  if (!m_arming_client.exists()) {
    set_response(false, m_arming_client.getService() + " does not exist!");
    return true;
  }

  // Check if takeoff client exists
  if (!m_takeoff_client.exists()) {
    set_response(false, m_takeoff_client.getService() + " does not exist!");
    return true;
  }

  // Check if odometry is alive
  if (!m_odom_handler->isResponsive()) {
    set_response(false, m_odom_handler->getName() + " unresponsive!");
    return true;
  }

  // Check if mavros state
  if (!m_mavros_state_handler->isResponsive()) {
    set_response(false, m_mavros_state_handler->getName() + " unresponsive!");
    return true;
  }

  // Check the carrot state
  if (!m_carrot_state_handler->isResponsive()) {
    set_response(false, m_carrot_state_handler->getName() + " unresponsive!");
    return true;
  }

  // At this point all of the takeoff components are active and takeoff can be started
  // -------------------------------------------

  // Check if the UAV is already armed
  if (m_mavros_state_handler->getData().armed) {
    set_response(false, "UAV is already armed, aborting");
    return true;
  }

  // Set the UAV to GUIDED_NOGPS
  if (req.set_offboard)
  {
    mavros_msgs::SetMode guided_request;
    guided_request.request.custom_mode = OFFBOARD_MODE;
    guided_request.request.base_mode   = 0;
    auto mode_call_success             = m_mode_client.call(guided_request);
    if (!mode_call_success || !guided_request.response.mode_sent) {
      set_response(false, std::string(OFFBOARD_MODE) + " mode switch failed!");
      return true;
    }
  }

  // Wait until the mode actually switches
  ros::Duration wait(WAIT_DURATION);
  while (ros::ok()) {
    wait.sleep();
    if (m_mavros_state_handler->getData().mode == OFFBOARD_MODE) { break; }
    auto status = check_elapsed_time("waiting for " + std::string(OFFBOARD_MODE));
    if (!std::get<0>(status)) {
      set_response(false, std::get<1>(status));
      return true;
    }
  }

  // Activate carrot reference
  if (req.enable_carrot) {
    sensor_msgs::Joy carrot_activation_msg;
    carrot_activation_msg.buttons = CARROT_ACTIVATION_VECTOR;
    m_joy_publisher.publish(carrot_activation_msg);
  }

  // Wait for correct pre-takeoff state
  while (ros::ok()) {
    wait.sleep();
    if (m_carrot_state_handler->getData().data == PRE_TAKEOFF_STATUS) { break; }
    auto status = check_elapsed_time("waiting for " + std::string(PRE_TAKEOFF_STATUS));
    if (!std::get<0>(status)) {
      set_response(false, std::get<1>(status));
      return true;
    }
  }

  // Arm the UAV
  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;
  auto arm_call_sucess      = m_arming_client.call(arm_command);
  if (!arm_call_sucess) {
    set_response(false, "UAV arm failed!");
    return true;
  }

  // Check if the UAV is actually armed.
  while (ros::ok()) {
    wait.sleep();
    if (m_mavros_state_handler->getData().armed) { break; }
    auto status = check_elapsed_time("waiting for UAV arm.");
    if (!std::get<0>(status)) {
      set_response(false, std::get<1>(status));
      return true;
    }
  }

  // UAV takeoff request
  uav_ros_msgs::TakeOff takeoff_request;
  takeoff_request.request.rel_alt = req.rel_alt;
  auto takeoff_call_success       = m_takeoff_client.call(takeoff_request);
  if (!takeoff_call_success) {
    set_response(false, "Takeoff call failed");
    return true;
  }

  if (!takeoff_request.response.success) {
    set_response(false,
                 "Takeoff failed with response: " + takeoff_request.response.message);
    return true;
  }

  set_response(true, "Takeoff successful");
  return true;
}

bool uav_ros_control::UAVManager::land_cb(uav_ros_msgs::Land::Request&  req,
                                          uav_ros_msgs::Land::Response& resp)
{
  const auto set_response = [&](bool status, std::string message) {
    resp.message = message;
    resp.success = status;
  };

  // Check if odometry is alive
  if (!m_odom_handler->isResponsive()) {
    set_response(false, m_odom_handler->getName() + " unresponsive!");
    return true;
  }

  // Check if mavros state
  if (!m_mavros_state_handler->isResponsive()) {
    set_response(false, m_mavros_state_handler->getName() + " unresponsive!");
    return true;
  }

  // Check the carrot state
  if (!m_carrot_state_handler->isResponsive()) {
    set_response(false, m_carrot_state_handler->getName() + " unresponsive!");
    return true;
  }

  // Check if land client exists
  if (!m_land_client.exists()) {
    set_response(false, m_land_client.getService() + " does not exist!");
    return true;
  }

  // Check if we are in OFFBOARD_MODE
  if (m_mavros_state_handler->getData().mode != OFFBOARD_MODE) {
    set_response(false, "UAV not in " + std::string(OFFBOARD_MODE));
    return true;
  }

  // Check if we are in position hold
  if (m_carrot_state_handler->getData().data != POSITION_HOLD) {
    set_response(false, "Carrot not in " + std::string(POSITION_HOLD));
    return true;
  }

  // Clear the mission if it exists
  if (m_clear_mission.exists()) {

    if (req.force_land)
    {
      std_srvs::SetBool clear_mission;
      clear_mission.request.data = true;

      auto clear_mission_response = m_clear_mission.call(clear_mission);
      if (!clear_mission_response) {
        set_response(false, "Unable to cancel the mission");
        return true;
      }

      if (!clear_mission.response.success) {
        set_response(false, "Cancelling mission request failed");
        return true;
      }
    }
    else 
    {
      //TODO(lmark): Message to future lovro
      //CarrotReference (or UAVControlManager) should not be in HOLD mode while
      //taking off / landing, since the tracker does not know if the UAV is landing or not.
      //Solution: Make a LANDOFF mode.
      set_response(false, "Please set 'force_disarm' to true if using missions.");
      return true;
    }

  }

  // Check if trajectory is being executed
  const auto tracker_status = m_tracker_state_handler->getData().data;
  if (tracker_status == TRACKER_ACTIVE) {
    if (req.force_land) {
      // Force disable the trajectory
      if (!m_tracker_reset_client.exists()) {
        set_response(false, "trying to force the landing, but unable to stop tracker.");
        return true;
      }

      std_srvs::Empty reset_request;
      auto            reset_success = m_tracker_reset_client.call(reset_request);
      if (!reset_success) {
        set_response(false, "trying to force the landing, but unable to stop tracker.");
        return true;
      }

      // Trajectory should be stopped at this point :)
      // TODO(lmark): Should probably add a different service type for stopping the
      // trajectory

    } else {
      // We do not want to force landing if a trajectory is executed
      set_response(false, "Unable to land, tracker is active");
      return false;
    }
  }

  // From this point everything is ready for landing
  // ----------------------------------------------

  std_srvs::SetBool land_request;
  land_request.request.data = true;
  auto land_success         = m_land_client.call(land_request);
  if (!land_success) {
    set_response(false, "Unable to request land.");
    return true;
  }

  if (!land_request.response.success) {
    set_response(false, "Land failed with message " + land_request.response.message);
    return true;
  }

  set_response(true, "Land successful");
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::UAVManager, nodelet::Nodelet)
