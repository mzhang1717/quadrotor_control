//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_input/ControllerBase.h>
#include <dynamic_reconfigure/server.h>
#include <quadrotor_input/AltitudeControllerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/BatteryStatus.h>
#include <control_toolbox/pid.h>
#include <boost/circular_buffer.hpp>

#ifndef ALTITUDE_CONTROLLER_H
#define ALTITUDE_CONTROLLER_H

#define MEAS_BUFFER_SIZE 5

/// Altitude controller that controls throttle to achieve a height setpoint with a PID controller.
/// Subscribes to a pose message (geometry_msgs/PoseStamped), a battery message (quadrotor_msgs/BatteryStatus)
/// and optionally a twist message (geometry_msgs/TwistStamped)
class AltitudeController : public ControllerBase
{
public:
  AltitudeController();

protected:
  
  /// Resets controllers
  virtual bool Init();
  
  /// Feeds PID controller with actual and desired heights, generates radio control message
  virtual void Compute(quadrotor_msgs::RadioControl& rcMsg, quadrotor_msgs::ControllerError& errorMsg);
  
  /// Callback called by the reconfigure program to set gains and limits
  void ReconfigureCallback(quadrotor_input::AltitudeControllerConfig &config, uint32_t level);
  
  /// Callback called by the pose subscriber
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
  
  /// Callback called by the velocity subscriber
  void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg);
  
  /// Callback called by the battery subscriber
  void BatteryCallback(const quadrotor_msgs::BatteryStatus::ConstPtr& batteryMsg);
  
  /// Calculate the derivative of the error, method depends on availability of velocity message
  double CalcErrorDerivative();
  
  /// Calculate the average battery voltage over the measurement buffer
  double CalcBatteryVoltage();
    
  /// Calculate the base throttle amount needed to keep the aircraft hovering, linear curve based on voltage
  double CalcBaseThrottle(double voltage);
  
  /// Calculate the scaling factor for the PID output
  double CalcScale(double baseThrottle);
  
  double GetDesiredHeight();
  
  ros::Subscriber poseSub;      ///< Subscriber for pose
  ros::Subscriber velocitySub;  ///< Subscriber for velocity
  ros::Subscriber batterySub;   ///< Subscriber for battery
  
  // Debug publisher
  ros::Publisher altitudeDebugPub; 
  
  boost::circular_buffer<std::pair<ros::Time, double> > errorHistory;
  boost::circular_buffer<double> voltageHistory;
  
  double currentVelocity;
  double currentHeight;
  double desiredHeight;
  
  ros::Time lastHeightTime;
  ros::Time lastVelocityTime;
  ros::Time lastBatteryTime;
  
  ros::Duration heightTimeBuffer;
  ros::Duration batteryTimeBuffer;
  ros::Duration velocityTimeBuffer;
  
  double voltageSetpointStart;
  double voltageSetpointEnd;
  double baseThrottleSetpointStart;
  double baseThrottleSetpointEnd;
  
  double mass;
  
  dynamic_reconfigure::Server<quadrotor_input::AltitudeControllerConfig> reconfigureServer;
  
  double pidLimit;  ///< Limit of allowed PID command
  double cmdMax;  ///< Max overall command
  double cmdMin;  ///< Min overall command
  
  ros::Time lastControlTime; 

  control_toolbox::Pid zCtrl;  ///< The PID controller for height
  
};

#endif
