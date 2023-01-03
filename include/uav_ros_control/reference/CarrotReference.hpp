#ifndef CARROT_REFERENCE_H
#define CARROT_REFERENCE_H

#include "ros/forwards.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <uav_ros_msgs/TakeOff.h>

#include <uav_ros_control/reference/JoyControlInput.hpp>
#include <uav_ros_lib/topic_handler.hpp>

namespace uav_reference {

/**
 * @brief This enum defines states for the CarrotReference.
 *
 */
enum CarrotState { OFF, GROUNDED, TAKEOFF, CARROT, HOLD, LAND, EMERGENCY };
static const char* CarrotStateStr[] = { "OFF",  "GROUNDED", "TAKEOFF",  "CARROT",
                                        "HOLD", "LAND",     "EMERGENCY" };

std::ostream& operator<<(std::ostream& os, const CarrotState& s)
{
  os << CarrotStateStr[s];
  return os;
}

/**
 * "Carrot-on-a-Stick" reference publisher class. Reference is published
 * with respect to the global coordinate system.
 */
class CarrotReference : public uav_reference::JoyControlInput
{
public:
  constexpr static double CARROT_DT = 0.02;

  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   */
  CarrotReference(ros::NodeHandle&);
  virtual ~CarrotReference();

  /**
   * Update carrot position setpoint, with default joystick offset values.
   * Also update the yaw reference value.
   */
  void updateCarrot();

  /**
   * Publish carrot position setpoint as a Vector3 ROS message.
   */
  void publishCarrotSetpoint();

  /**
   * Check if carrot mode is entered. This method will reset carrot position
   * when entering carrot reference mode for the first time.
   */
  void updateCarrotStatus();

  /**
   * True if carrot is enabled otherwise false.
   */
  bool isCarrotEnabled();

  /**
   * True if position hold is enabled, otherwise false.
   */
  bool isHoldEnabled();

private:
  /**
   * Reset controller integrators.
   */
  bool resetIntegrators();

  /**
   * Initialize class parameters.
   */
  void initializeParameters();

  /**
   * Update x and y component of carrot position setpoint with the given value.
   *
   * @param - x carrot offset
   * @param - y carrot offset
   */
  void updateCarrotXY(double x, double y);

  /**
   * Update x component of carrot position setpoint with default joystick offset
   * value.
   */
  void updateCarrotXY();

  /**
   * Update z component of carrot position setpoint with the given value.
   *
   * @param - z carrot offset
   */
  void updateCarrotZ(double z);

  /**
   * Update z component of carrot position setpoint with default joystick offset
   * value.
   */
  void updateCarrotZ();

  /**
   * Update carrot Yaw component
   */
  void updateCarrotYaw();

  /**
   * Reset carrot trajectory point.
   */
  void resetCarrot();

  /**
   * Position hold service callback.OS
   */
  bool posHoldServiceCb(std_srvs::Empty::Request&  request,
                        std_srvs::Empty::Response& response);

  bool takeoffServiceCb(uav_ros_msgs::TakeOff::Request&  request,
                        uav_ros_msgs::TakeOff::Response& response);

  bool landServiceCb(std_srvs::SetBool::Request&  request,
                     std_srvs::SetBool::Response& response);

  /**
   * Callback function for Position reference. Works only during position hold
   * mode.
   */
  void positionRefCb(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& posMsg);

  /**
   * Odometry callback function. Used for extracting UAV yaw rotation.
   */
  void odomCb(const nav_msgs::OdometryConstPtr&);

  /** Carrot reference used for position hold. */
  trajectory_msgs::MultiDOFJointTrajectoryPoint _carrotPoint;

  /** UAV current position array. */
  std::array<double, 3> _uavPos{ 0.0, 0.0, 0.0 };

  /** Referent yaw angle. */
  double _carrotYaw = 0;

  /** Current UAV yaw angle */
  double _uavYaw = 0;

  /** Current carrot state */
  std::mutex  _carrotStateMutex;
  CarrotState _carrotState = CarrotState::OFF;
  void        setCarrotState(const CarrotState& new_state);
  CarrotState getCarrotState();

  /** Index used for enabling carrot mode */
  int _carrotEnabledIndex = -1;

  /** Carrot enable button value, 0 or 1 **/
  int _carrotEnabledValue = 1;

  /** Frame ID for the Carrot reference **/
  std::string _frameId;

  /* First pass flag - set carrot to odometry */
  bool _firstPass            = true;
  bool _carrotLandEnabled    = true;
  bool _landDisarmEnabled    = false;

  /** Define all Publishers */
  ros::Publisher _pubCarrotTrajectorySp;
  ros::Publisher _pubCarrotYawSp;
  ros::Publisher _pubUAVYaw;
  ros::Publisher _pubCarrotPose;
  ros::Publisher _pubCarrotStatus;

  /** Define all Subscribers. */
  ros::Subscriber                            _subOdom;
  ros::Subscriber                            _subPosHoldRef;
  ros_util::TopicHandler<mavros_msgs::State> m_handlerState;

  /** Define all the services */
  ros::ServiceServer _servicePoisitionHold, _serviceTakeoff, _serviceLand;

  /* Reset integrator client */
  ros::ServiceClient _intResetClient, _setModeToLandClient, _forceDisarmClient;

  /* Define timers */
  ros::Timer _carrotTakeoffTimer;
  double     _takeoffSpeed             = 0.2;
  double     _initialTakeoffHeight     = 0.3;
  double     _takeoff_altitude_request = 2.0;
  void       takeoff_loop(const ros::TimerEvent& e);

  ros::Timer _carrotLandTimer;
  int        _landCounter           = 0;
  double     _lastAltDifference     = 0;
  double     _landSpeed             = 0.3;
  double     _land_altitude_request = 0.0;
  void       land_loop(const ros::TimerEvent& e);
};

/**
 * Run default Carrot Reference publishing node program.
 *
 * @param cc - Reference to CarrotReference object
 * @param nh - Given NodeHandle
 */
void runDefault(uav_reference::CarrotReference& cc, ros::NodeHandle& nh);
}// namespace uav_reference

#endif /** CARROT_REFERENCE_H */
