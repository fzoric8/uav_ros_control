#include "uav_manager.hpp"

void uav_ros_control::UAVManager::onInit()
{
  ROS_INFO("[UavManager] Initialized");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::UAVManager, nodelet::Nodelet)