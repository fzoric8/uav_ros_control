#include <uav_ros_control/control/CascadePID.hpp>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/OverrideRCIn.h>

#define CARROT_OFF "OFF"
#define CARROT_GROUNDED "GROUNDED"

// Define all parameter paths here
#define PID_X_PARAM "control/pos_x"
#define PID_Y_PARAM "control/pos_y"
#define PID_Z_PARAM "control/pos_z"
#define PID_VX_PARAM "control/vel_x"
#define PID_VY_PARAM "control/vel_y"
#define PID_VZ_PARAM "control/vel_z"
#define PID_YAW_RATE_PARAM "control/yaw_rate"

#define FFGAIN_VEL_X_PARAM "control/ff_gain/velocity/x"
#define FFGAIN_VEL_Y_PARAM "control/ff_gain/velocity/y"
#define FFGAIN_VEL_Z_PARAM "control/ff_gain/velocity/z"

#define FFGAIN_ACC_X_PARAM "control/ff_gain/acceleration/x"
#define FFGAIN_ACC_Y_PARAM "control/ff_gain/acceleration/y"
#define FFGAIN_ACC_Z_PARAM "control/ff_gain/acceleration/z"

#define HOVER_PARAM "control/hover"
#define GRAVITY_ACCELERATION 9.8;

uav_controller::CascadePID::CascadePID(ros::NodeHandle& nh)
  : _posXPID{ new PID("Position - x") }, _posYPID{ new PID("Position - y") },
    _posZPID{ new PID("Position - z") }, _velXPID{ new PID("Velocity - x") },
    _velYPID{ new PID("Velocity - y") }, _velZPID{ new PID("Velocity - z") },
    _yawRatePID{ new PID("Yaw rate") }, uav_controller::ControlBase(nh)
{
  // Initialize class parameters
  initializeParameters(nh);
  _velRefPub  = nh.advertise<geometry_msgs::Vector3>("carrot/velocity", 1);
  _velCurrPub = nh.advertise<geometry_msgs::Vector3>("uav/velocity", 1);
  _carrotStateSub =
    nh.subscribe("carrot/status", 1, &uav_controller::CascadePID::carrotStatusCb, this);

  // Setup dynamic reconfigure server
  uav_ros_control::PositionControlParametersConfig posConfig;
  setPositionReconfigureParams(posConfig);
  _posConfigServer.updateConfig(posConfig);
  _posParamCallback =
    boost::bind(&uav_controller::CascadePID::positionParamsCb, this, _1, _2);
  _posConfigServer.setCallback(_posParamCallback);

  // Initialize position hold service
  _serviceResetIntegrators = nh.advertiseService(
    "reset_integrator", &uav_controller::CascadePID::intResetServiceCb, this);
}

bool uav_controller::CascadePID::intResetServiceCb(std_srvs::Empty::Request&  request,
                                                   std_srvs::Empty::Response& response)
{
  ROS_WARN("Resetting all PID controllers");
  resetPositionPID();
  resetVelocityPID();
  _yawRatePID->resetIntegrator();
  return true;
}

bool uav_controller::CascadePID::activationPermission()
{
  return !(_carrotStatus == CARROT_OFF || _carrotStatus == CARROT_GROUNDED);
}

void uav_controller::CascadePID::carrotStatusCb(const std_msgs::StringConstPtr& msg)
{
  _carrotStatus = msg->data;
}

void uav_controller::CascadePID::positionParamsCb(
  uav_ros_control::PositionControlParametersConfig& configMsg,
  uint32_t                                          level)
{
  ROS_WARN("CascadePID::parametersCallback");

  _yawRatePID->set_kp(configMsg.yaw_rate_p);
  _yawRatePID->set_ki(configMsg.yaw_rate_i);
  _yawRatePID->set_kd(configMsg.yaw_rate_d);
  _yawRatePID->set_lim_high(configMsg.yaw_rate_lim_high);
  _yawRatePID->set_lim_low(configMsg.yaw_rate_lim_low);

  _posYPID->set_kp(configMsg.k_p_xy);
  _posYPID->set_kd(configMsg.k_d_xy);
  _posYPID->set_ki(configMsg.k_i_xy);
  _posYPID->set_lim_high(configMsg.lim_high_xy);
  _posYPID->set_lim_low(configMsg.lim_low_xy);

  _velYPID->set_kp(configMsg.k_p_vxy);
  _velYPID->set_kd(configMsg.k_d_vxy);
  _velYPID->set_ki(configMsg.k_i_vxy);
  _velYPID->set_lim_high(configMsg.lim_high_vxy);
  _velYPID->set_lim_low(configMsg.lim_low_vxy);

  _posXPID->set_kp(configMsg.k_p_xy);
  _posXPID->set_kd(configMsg.k_d_xy);
  _posXPID->set_ki(configMsg.k_i_xy);
  _posXPID->set_lim_high(configMsg.lim_high_xy);
  _posXPID->set_lim_low(configMsg.lim_low_xy);

  _velXPID->set_kp(configMsg.k_p_vxy);
  _velXPID->set_kd(configMsg.k_d_vxy);
  _velXPID->set_ki(configMsg.k_i_vxy);
  _velXPID->set_lim_high(configMsg.lim_high_vxy);
  _velXPID->set_lim_low(configMsg.lim_low_vxy);

  _posZPID->set_kp(configMsg.k_p_z);
  _posZPID->set_kd(configMsg.k_d_z);
  _posZPID->set_ki(configMsg.k_i_z);
  _posZPID->set_lim_high(configMsg.lim_high_z);
  _posZPID->set_lim_low(configMsg.lim_low_z);

  _velZPID->set_kp(configMsg.k_p_vz);
  _velZPID->set_kd(configMsg.k_d_vz);
  _velZPID->set_ki(configMsg.k_i_vz);
  _velZPID->set_lim_high(configMsg.lim_high_vz);
  _velZPID->set_lim_low(configMsg.lim_low_vz);

  _ffGainVelocityX = configMsg.ff_vel_x;
  _ffGainVelocityY = configMsg.ff_vel_y;
  _ffGainVelocityZ = configMsg.ff_vel_z;

  _ffGainAccelerationX = configMsg.ff_acc_x;
  _ffGainAccelerationY = configMsg.ff_acc_y;
  _ffGainAccelerationZ = configMsg.ff_acc_z;

  _hoverThrust = configMsg.hover;
}

void uav_controller::CascadePID::setPositionReconfigureParams(
  uav_ros_control::PositionControlParametersConfig& config)
{
  ROS_WARN("CascadePID::setReconfigureParameters");

  config.yaw_rate_p        = _yawRatePID->get_kp();
  config.yaw_rate_i        = _yawRatePID->get_ki();
  config.yaw_rate_d        = _yawRatePID->get_kd();
  config.yaw_rate_lim_high = _yawRatePID->get_lim_high();
  config.yaw_rate_lim_low  = _yawRatePID->get_lim_low();

  config.k_p_xy      = _posYPID->get_kp();
  config.k_i_xy      = _posYPID->get_ki();
  config.k_d_xy      = _posYPID->get_kd();
  config.lim_low_xy  = _posYPID->get_lim_low();
  config.lim_high_xy = _posYPID->get_lim_high();

  config.k_p_vxy      = _velYPID->get_kp();
  config.k_i_vxy      = _velYPID->get_ki();
  config.k_d_vxy      = _velYPID->get_kd();
  config.lim_low_vxy  = _velYPID->get_lim_low();
  config.lim_high_vxy = _velYPID->get_lim_high();

  config.k_p_z      = _posZPID->get_kp();
  config.k_i_z      = _posZPID->get_ki();
  config.k_d_z      = _posZPID->get_kd();
  config.lim_low_z  = _posZPID->get_lim_low();
  config.lim_high_z = _posZPID->get_lim_high();

  config.k_p_vz      = _velZPID->get_kp();
  config.k_i_vz      = _velZPID->get_ki();
  config.k_d_vz      = _velZPID->get_kd();
  config.lim_low_vz  = _velZPID->get_lim_low();
  config.lim_high_vz = _velZPID->get_lim_high();

  config.ff_vel_x = _ffGainVelocityX;
  config.ff_vel_y = _ffGainVelocityY;
  config.ff_vel_z = _ffGainVelocityZ;
  config.ff_acc_x = _ffGainAccelerationX;
  config.ff_acc_y = _ffGainAccelerationY;
  config.ff_acc_z = _ffGainAccelerationZ;

  config.hover = _hoverThrust;
}

void uav_controller::CascadePID::initializeParameters(ros::NodeHandle& nh)
{
  ROS_WARN("CascadePID::initializeParameters()");

  _posYPID->initializeParameters(nh, PID_Y_PARAM);
  _velYPID->initializeParameters(nh, PID_VY_PARAM);
  _posXPID->initializeParameters(nh, PID_X_PARAM);
  _velXPID->initializeParameters(nh, PID_VX_PARAM);
  _posZPID->initializeParameters(nh, PID_Z_PARAM);
  _velZPID->initializeParameters(nh, PID_VZ_PARAM);
  _yawRatePID->initializeParameters(nh, PID_YAW_RATE_PARAM);

  bool initialized = nh.getParam(HOVER_PARAM, _hoverThrust)
                     && nh.getParam(FFGAIN_VEL_X_PARAM, _ffGainVelocityX)
                     && nh.getParam(FFGAIN_VEL_Y_PARAM, _ffGainVelocityY)
                     && nh.getParam(FFGAIN_VEL_Z_PARAM, _ffGainVelocityZ)
                     && nh.getParam(FFGAIN_ACC_X_PARAM, _ffGainAccelerationX)
                     && nh.getParam(FFGAIN_ACC_Y_PARAM, _ffGainAccelerationY)
                     && nh.getParam(FFGAIN_ACC_Z_PARAM, _ffGainAccelerationZ);

  ROS_INFO("Hover thrust: %.2f", _hoverThrust);
  ROS_INFO("Feed-forward velocity gain: [%.2f, %.2f, %.2f]",
           _ffGainVelocityX,
           _ffGainVelocityY,
           _ffGainVelocityZ);
  ROS_INFO("Feed-forward acceleration gain: [%.2f, %.2f, %.2f]",
           _ffGainAccelerationX,
           _ffGainAccelerationY,
           _ffGainAccelerationZ);
  if (!initialized) {
    ROS_FATAL("CascadePID::initalizeParameters() - failed to initialize parameters");
    throw std::runtime_error("CascadedPID parameters not properly initialized.");
  }
}

void uav_controller::CascadePID::resetPositionPID()
{
  _posXPID->resetIntegrator();
  _posYPID->resetIntegrator();
  _posZPID->resetIntegrator();
}

void uav_controller::CascadePID::resetVelocityPID()
{
  _velXPID->resetIntegrator();
  _velYPID->resetIntegrator();
  _velZPID->resetIntegrator();
}

void uav_controller::CascadePID::calculateAttThrustSp(double dt, bool yaw_rate_control)
{
  // Calculate first row of PID controllers
  double velocityRefX = _posXPID->compute(
    getCurrentReference().transforms[0].translation.x, getCurrPosition()[0], dt);
  double velocityRefY = _posYPID->compute(
    getCurrentReference().transforms[0].translation.y, getCurrPosition()[1], dt);
  double velocityRefZ = _posZPID->compute(
    getCurrentReference().transforms[0].translation.z, getCurrPosition()[2], dt);

  // Add velocity feed-forward gains
  velocityRefX += _ffGainVelocityX * getCurrentReference().velocities[0].linear.x;
  velocityRefY += _ffGainVelocityY * getCurrentReference().velocities[0].linear.y;
  velocityRefZ += _ffGainVelocityZ * getCurrentReference().velocities[0].linear.z;

  // Calculate second row of PID controllers
  double roll   = -_velYPID->compute(velocityRefY, getCurrVelocity()[1], dt);
  double pitch  = _velXPID->compute(velocityRefX, getCurrVelocity()[0], dt);
  double thrust = _velZPID->compute(velocityRefZ, getCurrVelocity()[2], dt);
  thrust += _hoverThrust;

  // Add acceleration feed-forward gains
  roll += _ffGainAccelerationX * getCurrentReference().accelerations[0].linear.x
          / GRAVITY_ACCELERATION;
  pitch += _ffGainAccelerationY * getCurrentReference().accelerations[0].linear.y
           / GRAVITY_ACCELERATION;
  _yawRef += _ffGainAccelerationZ * getCurrentReference().accelerations[0].linear.z
             / GRAVITY_ACCELERATION;

  if (yaw_rate_control) {
    // Don't decouple roll - pitch - publish it in ENU frame
    setAttitudeSp(roll, pitch, 0);
  } else {
    // Decouple roll and pitch w.r.t. yaw - publish it in UAV body frame
    setAttitudeSp(cos(getCurrentYaw()) * roll + sin(getCurrentYaw()) * pitch,
                  cos(getCurrentYaw()) * pitch - sin(getCurrentYaw()) * roll,
                  _yawRef);
  }

  setThrustSp(thrust);

  geometry_msgs::Vector3 newMsg;
  newMsg.x = velocityRefX;
  newMsg.y = velocityRefY;
  newMsg.z = velocityRefZ;
  _velRefPub.publish(newMsg);

  geometry_msgs::Vector3 vel;
  vel.x = getCurrVelocity()[0];
  vel.y = getCurrVelocity()[1];
  vel.z = getCurrVelocity()[2];
  _velCurrPub.publish(vel);
}

double uav_controller::CascadePID::getYawRef() { return _yawRef; }

double uav_controller::CascadePID::calculateYawRateSetpoint(double dt)
{
  auto                  yawRef = _yawRef;
  auto                  yawMv  = getCurrentYaw();
  static constexpr auto tol    = M_PI;

  // Compensate for wrapping to [-PI, PI]
  if (yawRef - yawMv > tol) {
    yawRef -= 2 * M_PI;
  } else if (yawRef - yawMv < -tol) {
    yawRef += 2 * M_PI;
  }
  return _yawRatePID->compute(yawRef, getCurrentYaw(), dt);
}

void uav_controller::runDefault(uav_controller::CascadePID& cascadeObj,
                                ros::NodeHandle&            nh)
{
  double    rate = 50;
  double    dt   = 1.0 / rate;
  ros::Rate loopRate(rate);
  ROS_WARN_ONCE(
    "[uav_controller::runDefault]: Control node for Ardupilot firmware is active!");

  while (ros::ok()) {
    ros::spinOnce();
    if (cascadeObj.activationPermission()) {
      cascadeObj.calculateAttThrustSp(dt);
      cascadeObj.publishAttitudeTarget(MASK_IGNORE_RPY_RATE);
    } else {
      ROS_FATAL_THROTTLE(2, "CascadePID::runDefault - controller inactive");
    }
    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}

void uav_controller::runDefault_px4(uav_controller::CascadePID& cascadeObj,
                                    ros::NodeHandle&            nh)
{
  double    rate = 50;
  double    dt   = 1.0 / rate;
  ros::Rate loopRate(rate);
  ROS_WARN_ONCE(
    "[uav_controller::runDefault_px4]: Control node for PX4 firmware is active!");

  while (ros::ok()) {
    ros::spinOnce();
    cascadeObj.calculateAttThrustSp(dt);
    cascadeObj.publishAttitudeTarget(MASK_IGNORE_RPY_RATE);
    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}

void uav_controller::runDefault_yawrate(uav_controller::CascadePID& cascadeObj,
                                        ros::NodeHandle&            nh)
{
  double    rate = 50;
  double    dt   = 1.0 / rate;
  ros::Rate loopRate(rate);
  ROS_WARN_ONCE(
    "[uav_controller::runDefault_yawrate]: Control node for Ardupilot firmware is "
    "active!");

  while (ros::ok()) {
    ros::spinOnce();
    if (cascadeObj.activationPermission()) {
      cascadeObj.calculateAttThrustSp(dt);
      cascadeObj.overrideYawTarget(0);
      const auto yawRateSetpoint = cascadeObj.calculateYawRateSetpoint(dt);
      cascadeObj.publishAttitudeTarget(MASK_IGNORE_RP_RATE, yawRateSetpoint);
    } else {
      ROS_FATAL_THROTTLE(2, "CascadePID::runDefault_yawrate - controller inactive");

      ROS_WARN_THROTTLE(2, "CascadePID::runDefault - publishing zero to thrust");
      cascadeObj.overrideRollTarget(0);
      cascadeObj.overridePitchTarget(0);
      cascadeObj.overrideYawTarget(0);
      cascadeObj.setThrustSp(0);
      cascadeObj.publishAttitudeTarget(MASK_IGNORE_RP_RATE, 0);
    }
    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}

void uav_controller::runDefault_yawrate_px4(uav_controller::CascadePID& cascadeObj,
                                            ros::NodeHandle&            nh)
{
  double    rate = 50;
  double    dt   = 1.0 / rate;
  ros::Rate loopRate(rate);
  bool      yaw_rate_control_enabled = true;
  ROS_WARN_ONCE(
    "[uav_controller::runDefault_yawrate_px4]: Control node for PX4 firmware is active!");

  while (ros::ok()) {
    ros::spinOnce();
    cascadeObj.calculateAttThrustSp(dt, yaw_rate_control_enabled);
    const auto yawRateSetpoint = cascadeObj.calculateYawRateSetpoint(dt);
    cascadeObj.publishAttitudeTarget(MASK_IGNORE_RP_RATE, yawRateSetpoint);
    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}

void uav_controller::runDefault_tilt_control(uav_controller::CascadePID& cascadeObj,
                                             ros::NodeHandle&            nh)
{
  constexpr double rate = 50;
  double           dt   = 1.0 / rate;
  ros::Rate        loopRate(rate);

  int  default_pwm_roll, default_pwm_pitch, roll_tilt_channel, pitch_tilt_channel;
  bool initialized = nh.getParam("control/roll_tilt_channel", roll_tilt_channel)
                     && nh.getParam("control/pitch_tilt_channel", pitch_tilt_channel)
                     && nh.getParam("control/default_pwm_roll", default_pwm_roll)
                     && nh.getParam("control/default_pwm_pitch", default_pwm_pitch);

  if (!initialized) {
    ROS_FATAL(
      "[uav_controller::runDefault_tilt_control] Parameter initialization failed.");
    throw std::runtime_error("Parameter initialization failed.");
  }

  ros::Publisher overridePub =
    nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1);
  std::vector<int> override_indices = { 0, 0, 0, 0, 0, 0, 0, 0 };

  ROS_WARN_ONCE(
    "[uav_controller::runDefault_tilt_control] Control node for Ardupilot firmware is "
    "active!");

  while (ros::ok()) {
    ros::spinOnce();
    if (cascadeObj.activationPermission()) {
      cascadeObj.calculateAttThrustSp(dt);

      // Set tilt PWM values acoording to the calculated roll and pitch setpoint
      const auto& att_thrust_sp            = cascadeObj.getAttThrustSp();
      override_indices[roll_tilt_channel]  = default_pwm_roll - att_thrust_sp[0];
      override_indices[pitch_tilt_channel] = default_pwm_pitch + att_thrust_sp[1];

      // Roll and Pitch setpoints are forwarded as attitude command
      auto rpy_ref = cascadeObj.getReferentRPY();
      ROS_INFO_STREAM_THROTTLE(2.0, rpy_ref[0] << ", " << rpy_ref[1]);
      cascadeObj.overrideRollTarget(rpy_ref[0]);
      cascadeObj.overridePitchTarget(rpy_ref[1]);

      // Do yaw rate control
      cascadeObj.overrideYawTarget(0);
      const auto yawRateSetpoint = cascadeObj.calculateYawRateSetpoint(dt);

      cascadeObj.publishAttitudeTarget(MASK_IGNORE_RP_RATE, yawRateSetpoint);
    } else {

      // Turn off Tilt override
      override_indices = { 0, 0, 0, 0, 0, 0, 0, 0 };
      ROS_FATAL_THROTTLE(2, "CascadePID::runDefault_tilt_control - controller inactive");
    }

    // Publish
    mavros_msgs::OverrideRCIn newMsg;
    std::copy(override_indices.begin(), override_indices.end(), newMsg.channels.elems);
    overridePub.publish(newMsg);

    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}
