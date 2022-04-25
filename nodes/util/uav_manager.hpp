#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>

namespace uav_ros_control {

class UAVManager : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
};

}// namespace uav_ros_control