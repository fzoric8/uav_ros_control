#include "ros/forwards.h"
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_control/reference/CarrotReference.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>
#include <mavros_msgs/CommandLong.h>

#define CARROT_OFF "OFF"
#define CARROT_ON_LAND "CARROT_ON_LAND"
#define CARROT_ON_AIR "CARROT_ON_AIR"
#define POS_HOLD "HOLD"

uav_reference::CarrotReference::CarrotReference(ros::NodeHandle& nh)
  : m_handlerState(nh, "mavros/state"), uav_reference::JoyControlInput(nh)
{
  // Define Publishers
  _pubCarrotTrajectorySp =
    nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("carrot/trajectory", 1);
  _pubCarrotYawSp  = nh.advertise<std_msgs::Float64>("carrot/yaw", 1);
  _pubUAVYaw       = nh.advertise<std_msgs::Float64>("uav/yaw", 1);
  _pubCarrotPose   = nh.advertise<geometry_msgs::PoseStamped>("carrot/pose", 1);
  _pubCarrotStatus = nh.advertise<std_msgs::String>("carrot/status", 1);

  // Define Subscribers
  _subOdom = nh.subscribe("odometry", 1, &uav_reference::CarrotReference::odomCb, this);
  _subPosHoldRef = nh.subscribe(
    "position_hold/trajectory", 1, &uav_reference::CarrotReference::positionRefCb, this);

  // Initialize position hold service
  _servicePoisitionHold = nh.advertiseService(
    "position_hold", &uav_reference::CarrotReference::posHoldServiceCb, this);
  _serviceTakeoff = nh.advertiseService(
    "takeoff", &uav_reference::CarrotReference::takeoffServiceCb, this);

  _serviceLand =
    nh.advertiseService("land", &uav_reference::CarrotReference::landServiceCb, this);

  _setModeToLandClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  _intResetClient    = nh.serviceClient<std_srvs::Empty>("reset_integrator");
  _forceDisarmClient = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  initializeParameters();

  if (!_manualTakeoffEnabled) {
    ROS_WARN("CarrotReference - Automatic takeoff enabled, position hold disabled");
    _positionHold = false;
  }

  // Initialize references
  _carrotPoint.transforms    = std::vector<geometry_msgs::Transform>(1);
  _carrotPoint.velocities    = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.accelerations = std::vector<geometry_msgs::Twist>(1);

  _carrotTakeoffTimer = nh.createTimer(ros::Rate(1 / CARROT_DT),
                                       &uav_reference::CarrotReference::takeoff_loop,
                                       this,
                                       false,
                                       false);

  _carrotLandTimer = nh.createTimer(ros::Rate(1 / CARROT_DT),
                                    &uav_reference::CarrotReference::land_loop,
                                    this,
                                    false,
                                    false);
}

uav_reference::CarrotReference::~CarrotReference() {}

bool uav_reference::CarrotReference::posHoldServiceCb(std_srvs::Empty::Request&  request,
                                                      std_srvs::Empty::Response& response)
{
  if (!_carrotEnabled) {
    ROS_FATAL(
      "CarrotReference::posHoldServiceCb - unable to activate POSITION_HOLD, carrot not "
      "enabled");
    return true;
  }

  if (_carrotOnLand) {
    ROS_FATAL(
      "CarrotReference::posHoldServiceCb - unable to activate POSITION_HOLD, carrot on "
      "land");
    return true;
  }

  if (!_positionHold) {
    ROS_WARN("CarrotReference::posHoldServiceCb - Position hold enabled");
    _positionHold = true;
    resetCarrot();
  }

  return true;
}

bool uav_reference::CarrotReference::landServiceCb(std_srvs::SetBool::Request&  request,
                                                   std_srvs::SetBool::Response& response)
{
  const auto set_response = [&response](bool success) { response.success = success; };

  if (_manualTakeoffEnabled) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, in MANUAL_TAKEOFF.");
    set_response(false);
    return true;
  }

  if (m_handlerState.getData().mode != "GUIDED_NOGPS") {
    ROS_FATAL(
      "CarrotReference::landServiceCb - unable to land, not in GUIDED_NOGPS mode");
    set_response(false);
    return true;
  }

  if (!_carrotEnabled) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, CARROT disabled");
    set_response(false);
    return true;
  }

  if (_carrotOnLand || !_takeoffHappened) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, UAV on land");
    set_response(false);
    return true;
  }

  if (!request.data) {
    ROS_FATAL("CarrotReference::landServiceCb - request to not land recieved.");
    set_response(false);
    return true;
  }

  if (_carrotLandEnabled) {

    // Can't land in carrot if position hold is not enabled
    if (!_positionHold) {
      ROS_FATAL(
        "CarrotReference::landServiceCb - Carrot Landing is enabled but position hold is "
        "disabled.");
      set_response(false);
      return true;
    }

    _landCounter       = 0;
    _lastAltDifference = 0;
    resetCarrot();
    _carrotLandTimer.start();
    ROS_INFO("CarrotReference::landServiceCb - Carrot Land started");
    set_response(true);
    return true;
  }

  // Assume we want to land at this point
  mavros_msgs::SetMode::Request  landMode_req;
  mavros_msgs::SetMode::Response landMode_resp;
  landMode_req.custom_mode = "LAND";

  if (!_setModeToLandClient.call(landMode_req, landMode_resp)) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to set LAND mode");
    set_response(false);
    return true;
  }

  ROS_INFO("Carrotreference::landServiceCb - LAND finished");
  set_response(true);
  return true;
}

bool uav_reference::CarrotReference::takeoffServiceCb(
  uav_ros_msgs::TakeOff::Request&  request,
  uav_ros_msgs::TakeOff::Response& response)
{
  const auto set_response = [&response](bool success) {
    response.success = success;
    if (success) {
      response.message = "Takeoff sucessful";
    } else {
      response.message = "Takeoff unsucessful";
    }
  };

  if (_manualTakeoffEnabled) {
    ROS_FATAL("CarrotReference::takeoffServiceCb - unable to takeoff, in MANUAL_TAKEOFF");
    set_response(false);
    return true;
  }

  // Check if the UAV is armed
  if (!m_handlerState.getData().armed) {
    ROS_FATAL("CarrotReference::takeoffServiceCb - unable to takeoff, not ARMED.");
    set_response(false);
    return true;
  }

  // Check if carrot is enabled
  if (!_carrotEnabled) {
    ROS_FATAL(
      "CarrotReference::takeoffServiceCb - unable to takeoff, carrot not enabled");
    set_response(false);
    return true;
  }

  if (!_carrotOnLand) {
    ROS_FATAL(
      "CarrotReference::takeoffServiceCb - unable to takeoff, not in CARROT_ON_LAND "
      "mode");
    set_response(false);
    return true;
  }

  resetCarrot();
  bool reset_success = resetIntegrators();
  if (!reset_success) {
    ROS_FATAL(
      "CarrotReference::takeoffServiceCb - unable to takeoff, failed to reset controller "
      "integrators.");
    set_response(false);
    return true;
  }

  _carrotOnLand                            = false;
  _positionHold                            = true;
  _takeoff_altitude_request                = _uavPos[2] + request.rel_alt;
  _carrotPoint.transforms[0].translation.z = _uavPos[2] + _initialTakeoffHeight;
  ROS_INFO("CarrotReference::takeoffServiceCb - enable position hold");
  _carrotTakeoffTimer.start();

  set_response(true);
  return true;
}

void uav_reference::CarrotReference::positionRefCb(
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& posMsg)
{
  if (_positionHold) {
    _carrotPoint = *posMsg;
    _carrotYaw   = ros_convert::calculateYaw(posMsg->transforms[0].rotation.x,
                                           posMsg->transforms[0].rotation.y,
                                           posMsg->transforms[0].rotation.z,
                                           posMsg->transforms[0].rotation.w);
    ROS_WARN_THROTTLE(5.0, "CarrotReference - Trajectory reference received");
  } else {
    ROS_FATAL(
      "CarrotReference - Trajectory reference recieved, but position hold mode is "
      "disabled.");
  }
}

bool uav_reference::CarrotReference::resetIntegrators()
{
  std_srvs::Empty::Request  req;
  std_srvs::Empty::Response resp;
  if (!_intResetClient.call(req, resp)) {
    ROS_FATAL("CarrotReference - Unable to reset integrators");
    return false;
  }

  ROS_INFO("Controller integrators reset.");
  return true;
}

void uav_reference::CarrotReference::updateCarrot()
{
  // Disable Position hold if carrot inputs exist
  if (_positionHold && isJoyActive()) {
    ROS_WARN("Position hold disabled - resetting carrot position");
    resetCarrot();
    _positionHold = false;
  }

  if (_positionHold) {
    ROS_INFO_THROTTLE(10.0,
                      "CarrotReference::update - listen to position_hold/trajectory.");
    return;
  }

  // Update carrot unless in position hold
  if (_carrotEnabled && !_carrotOnLand) {
    ROS_INFO_THROTTLE(10.0, "CarrotReference::update - joy update");
    updateCarrotXY();
    updateCarrotZ();
    updateCarrotYaw();
  } else {
    ROS_INFO_THROTTLE(10.0, "CarrotReference::update - reset to odom.");
    resetCarrot();
  }
}

void uav_reference::CarrotReference::resetCarrot()
{
  _carrotPoint.transforms                  = std::vector<geometry_msgs::Transform>(1);
  _carrotPoint.velocities                  = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.accelerations               = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.transforms[0].translation.x = _uavPos[0];
  _carrotPoint.transforms[0].translation.y = _uavPos[1];
  _carrotPoint.transforms[0].translation.z = _uavPos[2];

  // Set carrot orientation
  _carrotYaw = _uavYaw;
  tf2::Quaternion q;
  q.setEulerZYX(_carrotYaw, 0, 0);
  _carrotPoint.transforms[0].rotation.x = q.getX();
  _carrotPoint.transforms[0].rotation.y = q.getY();
  _carrotPoint.transforms[0].rotation.z = q.getZ();
  _carrotPoint.transforms[0].rotation.w = q.getW();
}


void uav_reference::CarrotReference::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  // Extract UAV yaw
  _uavYaw = ros_convert::calculateYaw(qx, qy, qz, qw);

  // Publish UAV yaw message
  std_msgs::Float64 uavYawMsg;
  uavYawMsg.data = _uavYaw;
  _pubUAVYaw.publish(uavYawMsg);

  // Extract UAV position
  _uavPos[0] = msg->pose.pose.position.x;
  _uavPos[1] = msg->pose.pose.position.y;
  _uavPos[2] = msg->pose.pose.position.z;

  if (_firstPass) {
    _firstPass = false;
    resetCarrot();
  }
}

void uav_reference::CarrotReference::updateCarrotYaw()
{
  // Update Carrot yaw angle and wrap to PI
  _carrotYaw += getYawSpManual();
  _carrotYaw = ros_convert::wrapMinMax(_carrotYaw, -M_PI, M_PI);

  tf2::Quaternion q;
  q.setEulerZYX(_carrotYaw, 0, 0);
  _carrotPoint.transforms[0].rotation.x = q.getX();
  _carrotPoint.transforms[0].rotation.y = q.getY();
  _carrotPoint.transforms[0].rotation.z = q.getZ();
  _carrotPoint.transforms[0].rotation.w = q.getW();
}

void uav_reference::CarrotReference::updateCarrotXY()
{
  updateCarrotXY(getXOffsetManual(), getYOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotZ()
{
  updateCarrotZ(getZOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotXY(double xOff, double yOff)
{
  // Adjust carrot position w.r.t. the global coordinate system
  _carrotPoint.transforms[0].translation.x += cos(-_uavYaw) * xOff + sin(-_uavYaw) * yOff;
  _carrotPoint.transforms[0].translation.y += cos(-_uavYaw) * yOff - sin(-_uavYaw) * xOff;
}

void uav_reference::CarrotReference::updateCarrotZ(double zOff)
{
  _carrotPoint.transforms[0].translation.z += zOff;
}

void uav_reference::CarrotReference::publishCarrotSetpoint()
{
  // Publish PoseStamped carrot reference
  _pubCarrotTrajectorySp.publish(_carrotPoint);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id    = _frameId;
  pose.header.stamp       = ros::Time::now();
  pose.pose.position.x    = _carrotPoint.transforms[0].translation.x;
  pose.pose.position.y    = _carrotPoint.transforms[0].translation.y;
  pose.pose.position.z    = _carrotPoint.transforms[0].translation.z;
  pose.pose.orientation.x = _carrotPoint.transforms[0].rotation.x;
  pose.pose.orientation.y = _carrotPoint.transforms[0].rotation.y;
  pose.pose.orientation.z = _carrotPoint.transforms[0].rotation.z;
  pose.pose.orientation.w = _carrotPoint.transforms[0].rotation.w;
  _pubCarrotPose.publish(pose);

  // Publish referent yaw message
  std_msgs::Float64 yawRefMsg;
  yawRefMsg.data = ros_convert::calculateYaw(_carrotPoint.transforms[0].rotation);
  _pubCarrotYawSp.publish(yawRefMsg);
}

void uav_reference::CarrotReference::initializeParameters()
{
  ROS_WARN("CarrotReference::initializeParameters()");

  ros::NodeHandle nhPrivate("~");
  param_util::getParamOrThrow(nhPrivate, "carrot_index", _carrotEnabledIndex);
  param_util::getParamOrThrow(nhPrivate, "carrot_enable", _carrotEnabledValue);
  param_util::getParamOrThrow(nhPrivate, "carrot_land", _carrotLandEnabled);
  param_util::getParamOrThrow(nhPrivate, "manual_takeoff", _manualTakeoffEnabled);
  param_util::getParamOrThrow(nhPrivate, "carrot_frame_id", _frameId);
  param_util::getParamOrThrow(nhPrivate, "land_speed", _landSpeed);
  param_util::getParamOrThrow(nhPrivate, "initial_takeoff_height", _initialTakeoffHeight);
  param_util::getParamOrThrow(nhPrivate, "takeoff_speed", _takeoffSpeed);
  param_util::getParamOrThrow(nhPrivate, "land_disarm_enabled", _landDisarmEnabled);
}

void uav_reference::CarrotReference::updateCarrotStatus()
{
  // Check if we landed after takeoff happened in automatic takeoff mode
  if (!_manualTakeoffEnabled && _takeoffHappened && !m_handlerState.getData().armed) {
    ROS_WARN("CarrotReference::updateCarrotStatus - UAV disarmed, assume LAND happened");
    _takeoffHappened = false;
    if (_carrotEnabled) { _carrotOnLand = true; }
    resetCarrot();
  }

  // Detect enable button - rising edge
  if (getJoyButtons()[_carrotEnabledIndex] == _carrotEnabledValue && !_carrotEnabled) {
    if (_manualTakeoffEnabled || _takeoffHappened) {
      _carrotEnabled = true;
      _carrotOnLand  = false;
      ROS_INFO("CarrotReference::updateCarrotStatus - CARROT_ON_AIR enabled.");
      resetIntegrators();
      resetCarrot();
    } else if (!_carrotOnLand) {
      ROS_INFO("CarrotReference::updateCarrotStatus - CARROT_ON_LAND enabled.");
      _carrotEnabled = true;
      _carrotOnLand  = true;
      resetIntegrators();
      resetCarrot();
    }
  }

  // Detect enable button - falling edge
  if (getJoyButtons()[_carrotEnabledIndex] == (1 - _carrotEnabledValue)
      && _carrotEnabled) {
    _carrotEnabled = false;
    _positionHold  = false;
    _carrotOnLand  = false;
    resetIntegrators();
    ROS_INFO("CarrotRefernce::updateCarrotStatus - carrot disabled.\n");
  }

  // Publish carrot status.
  std_msgs::String status;
  if (_positionHold && _carrotEnabled) {
    status.data = POS_HOLD;
  } else if (!_positionHold && _carrotEnabled) {
    if (_carrotOnLand) {
      status.data = CARROT_ON_LAND;
    } else {
      status.data = CARROT_ON_AIR;
    }
  } else {
    status.data = CARROT_OFF;
  }

  _pubCarrotStatus.publish(status);
}

bool uav_reference::CarrotReference::isCarrotEnabled() { return _carrotEnabled; }

bool uav_reference::CarrotReference::isHoldEnabled() { return _positionHold; }

void uav_reference::CarrotReference::takeoff_loop(const ros::TimerEvent& e)
{

  if (!_positionHold) {
    ROS_WARN("CarrotReference::takeoff_loop - position hold disabled, aborting takeoff!");
    resetCarrot();
    _carrotTakeoffTimer.stop();
    return;
  }

  ROS_INFO_THROTTLE(2.0, "CarrotReference::takeoff_loop");

  _carrotPoint.transforms[0].translation.z += _takeoffSpeed * CARROT_DT;

  if (_carrotPoint.transforms[0].translation.z >= _takeoff_altitude_request) {
    ROS_INFO("CarrotReference::takeoff_loop - takeoff happened.");
    _takeoffHappened = true;
    _carrotTakeoffTimer.stop();
  }
}

void uav_reference::CarrotReference::land_loop(const ros::TimerEvent& e)
{

  if (!_positionHold) {
    ROS_WARN("CarrotReference::land_loop - position hold disabled, aborting land!");
    resetCarrot();
    _carrotLandTimer.stop();
    return;
  }

  ROS_INFO_THROTTLE(2.0, "CarrotReference::land_loop");
  _carrotPoint.transforms[0].translation.z -= _landSpeed * CARROT_DT;

  double curr_difference = abs(_carrotPoint.transforms[0].translation.z - _uavPos[2]);
  if (curr_difference > 1 && curr_difference > _lastAltDifference) {
    _landCounter++;
    ROS_INFO("CarrotReference::land_loop - land counter at %d", _landCounter);
  } else {
    ROS_FATAL_THROTTLE(2.0, "Land counter reset");
    _landCounter = 0;
  }
  _lastAltDifference = curr_difference;

  if (!m_handlerState.getData().armed || _landCounter > 50) {
    ROS_INFO("CarrotReference::land_loop - land happened.");
    _takeoffHappened = false;
    _carrotOnLand    = true;
    _positionHold    = false;
    _carrotLandTimer.stop();

    if (_landDisarmEnabled) {
      mavros_msgs::CommandLong::Request  disarm_req;
      mavros_msgs::CommandLong::Response disarm_resp;

      disarm_req.broadcast    = false;
      disarm_req.command      = 400;
      disarm_req.confirmation = 0;
      disarm_req.param1       = 0;
      disarm_req.param2       = 21196;
      disarm_req.param3       = 0;
      disarm_req.param4       = 0;
      disarm_req.param5       = 0;
      disarm_req.param6       = 0;
      disarm_req.param7       = 0;

      if (!_forceDisarmClient.call(disarm_req, disarm_resp)) {
        ROS_FATAL("CarrotReference::land_loop - unable to call disarm client!");
        return;
      }

      if (disarm_resp.success) {
        ROS_INFO("CarrotReference::land_loop - UAV successfully disarmed!");
        return;
      }

      ROS_WARN("CarrotReference::land_loop - Disarm failed with message %d",
               disarm_resp.result);
    }
  }
}

void uav_reference::runDefault(uav_reference::CarrotReference& carrotRefObj,
                               ros::NodeHandle&                nh)
{
  double    rate = 1 / CarrotReference::CARROT_DT;
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    carrotRefObj.updateCarrotStatus();
    carrotRefObj.updateCarrot();
    carrotRefObj.publishCarrotSetpoint();
    loopRate.sleep();
  }
}
