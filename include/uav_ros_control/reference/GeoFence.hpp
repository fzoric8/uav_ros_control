#ifndef GEO_FENCE_H
#define GEO_FENCE_H

#include <ros/ros.h>
#include <uav_ros_lib/global_to_local.hpp>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include "ros/service_server.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <stdbool.h>
#include <cmath>

bool lineIntersection(geometry_msgs::Vector3  fixed_1,
                      geometry_msgs::Vector3  fixed_2,
                      geometry_msgs::Vector3  fixed_c,
                      geometry_msgs::Vector3  relative,
                      geometry_msgs::Vector3& new_reference)
{
  // Line AB represented as a1x + b1y = c1
  double a1 = fixed_2.y - fixed_1.y;
  double b1 = fixed_1.x - fixed_2.x;
  double c1 = a1 * (fixed_1.x) + b1 * (fixed_1.y);

  // Line CD represented as a2x + b2y = c2
  double a2 = relative.y - fixed_c.y;
  double b2 = fixed_c.x - relative.x;
  double c2 = a2 * (fixed_c.x) + b2 * (fixed_c.y);

  double determinant = a1 * b2 - a2 * b1;

  if (determinant == 0) {
    // The lines are parallel. This is simplified
    // by returning a pair of FLT_MAX
    return false;
  } else {
    new_reference.x = (b2 * c1 - b1 * c2) / determinant;
    new_reference.y = (a1 * c2 - a2 * c1) / determinant;
    return true;
  }
}

double dotProduct(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

double calcDistance(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2)
{
  return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}

double compareVectors(std::pair<geometry_msgs::Vector3, double> v1,
                      std::pair<geometry_msgs::Vector3, double> v2)
{
  return (v1.second < v2.second);
}

bool isPointOnLine(geometry_msgs::Vector3 point,
                   geometry_msgs::Vector3 v1,
                   geometry_msgs::Vector3 v2)
{

  double dx      = v2.x - v1.x;
  double dy      = v2.y - v1.y;
  double epsilon = 0.005 * (dx * dx + dy * dy);

  double pdp = (v1.x - point.x) * (v2.y - point.y) - (v1.y - point.y) * (v2.x - point.x);
  ROS_DEBUG("isPointOnLine: %.4f < %.4f", fabs(pdp), epsilon);

  return fabs(pdp) < epsilon;
}

template<typename T> inline bool less_or_equal(T lhs, T rhs)
{
  static constexpr auto TOL = 1e-4;
  return lhs < rhs || fabs(lhs - rhs) < TOL;
}

bool isPointOnLineSegment(geometry_msgs::Vector3 point,
                          geometry_msgs::Vector3 v1,
                          geometry_msgs::Vector3 v2)
{
  // If the proj is on the polygon, that is the closest point.
  if (!(

        (less_or_equal(v1.x, point.x) && less_or_equal(point.x, v2.x))
        || (less_or_equal(v2.x, point.x) && less_or_equal(point.x, v1.x)))) {
    ROS_DEBUG("x range: [%.2f <= %.2f && %.2f <= %.2f] || [%.2f <= %.2f && %.2f <= %.2f]",
              v1.x,
              point.x,
              point.x,
              v2.x,
              v2.x,
              point.x,
              point.x,
              v1.x);
    return false;
  }
  if (!((less_or_equal(v1.y, point.y) && less_or_equal(point.y, v2.y))
        || (less_or_equal(v2.y, point.y) && less_or_equal(point.y, v1.y)))) {
    ROS_DEBUG("y range: [%.2f <= %.2f && %.2f <= %.2f] || [%.2f <= %.2f && %.2f <= %.2f]",
              v1.y,
              point.y,
              point.y,
              v2.y,
              v2.y,
              point.y,
              point.y,
              v1.y);
    return false;
  }
  return isPointOnLine(point, v1, v2);
}

geometry_msgs::Vector3 findProjection(geometry_msgs::Vector3 v1,
                                      geometry_msgs::Vector3 v2,
                                      geometry_msgs::Vector3 p)
{
  // Project the current point on line between them.
  geometry_msgs::Vector3 e1;
  e1.x = v2.x - v1.x;
  e1.y = v2.y - v1.y;

  geometry_msgs::Vector3 e2;
  e2.x = p.x - v1.x;
  e2.y = p.y - v1.y;

  double dp  = e1.x * e2.x + e1.y * e2.y;
  double len = e1.x * e1.x + e1.y * e1.y;

  geometry_msgs::Vector3 proj;
  proj.x = v1.x + (dp * e1.x) / len;
  proj.y = v1.y + (dp * e1.y) / len;
  return proj;
}

double limitValue(double value, double min, double max)
{
  if (value > max) {
    return max;
  } else if (value < min) {
    return min;
  } else {
    return value;
  }
}

namespace uav_reference {
class GeoFence
{
public:
  GeoFence(ros::NodeHandle&, std::string filename);
  virtual ~GeoFence();

private:
  void referenceCb(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg);
  bool checkInside2D(geometry_msgs::Vector3 current);
  geometry_msgs::Vector3 findClosestPoint(geometry_msgs::Vector3 current);
  double                 limitZ(double z);
  int                    isLeft(geometry_msgs::Vector3 P0,
                                geometry_msgs::Vector3 P1,
                                geometry_msgs::Vector3 P2);

  tf_util::GlobalToLocal _global_to_local;// Object for converting GPS points
  geometry_msgs::Vector3 _last_valid_position;// Last received position within fence
  geometry_msgs::Vector3 _centroid;// Centroid of the allowed geo fence area
  std::vector<geometry_msgs::Vector3> _vertices;// GPS points definining fence polygon
  double                              _max_z;// Maximum allowed flying height
  double                              _min_z;// Minimum allowed flying height

  std::string     _frame_id;
  ros::Publisher  _pub;
  ros::Publisher  _fence_pose_pub;
  ros::Publisher  _fence_status_pub;
  ros::Subscriber _sub;

  bool               _is_active = false;
  ros::ServiceServer _activate_srv;
  bool activate_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);

  // Reference jump params
  double _max_jump;
  double _jump_constant;
  bool   _first_ref = false;
};

void runDefault(uav_reference::GeoFence& gf, ros::NodeHandle& nh);
}// namespace uav_reference
#endif
