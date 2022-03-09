//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_input/ControllerBase.h>
#include <dynamic_reconfigure/server.h>
#include <quadrotor_input/PositionControllerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/BatteryStatus.h>
#include <control_toolbox/pid.h>
#include <boost/circular_buffer.hpp>

#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#define MEAS_BUFFER_SIZE 5

/// Position controller that controls throttle and desired roll and pitch to achieve position (x, y, z) setpoints with PID controllers.
/// Subscribes to a pose message (geometry_msgs/PoseStamped) and a velocity message (geometry_msgs/TwistStamped)
class PositionController : public ControllerBase
{
public:
  PositionController();

protected:
  
  /// Resets controllers
  virtual bool Init();
  
  /// Feeds PID controller with actual and desired heights, generates radio control message
  virtual void Compute(quadrotor_msgs::RadioControl& rcMsg, quadrotor_msgs::ControllerError& errorMsg);
  
  /// Callback called by the reconfigure program to set gains and limits
  void ReconfigureCallback(quadrotor_input::PositionControllerConfig &config, uint32_t level);
  
  /// Callback called by the pose subscriber (if subscribed)
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
  
  void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg);
  
  void BatteryCallback(const quadrotor_msgs::BatteryStatus::ConstPtr& batteryMsg);
  
  geometry_msgs::Vector3 CalcErrorDerivative();
  
  double CalcBatteryVoltage();
    
  double CalcBaseThrottle(double voltage);
  
  double CalcScale(double baseThrottle);
  
  geometry_msgs::Point GetDesiredPosition();
  
  ros::Subscriber poseSub;
  ros::Subscriber velocitySub;
  ros::Subscriber batterySub;
  
  // Debug publisher
  ros::Publisher positionDebugPub; 

  boost::circular_buffer<std::pair<ros::Time, geometry_msgs::Vector3> > errorHistory;
  boost::circular_buffer<double> voltageHistory;
  
  geometry_msgs::Vector3 currentVelocity;
  geometry_msgs::Pose currentPose;
  //geometry_msgs::Point currentPosition;
  geometry_msgs::Point desiredPosition;
  
  ros::Time lastPositionTime;
  ros::Time lastVelocityTime;
  ros::Time lastBatteryTime;
  
  ros::Duration positionTimeBuffer;
  ros::Duration batteryTimeBuffer;
  ros::Duration velocityTimeBuffer;
  
  double voltageSetpointStart;
  double voltageSetpointEnd;
  double baseThrottleSetpointStart;
  double baseThrottleSetpointEnd;
  
  double mass;
  bool xyActive;  // for debugging the xy control
  
  dynamic_reconfigure::Server<quadrotor_input::PositionControllerConfig> reconfigureServer;
  
  geometry_msgs::Vector3 pidLimit;  ///< Limit of allowed PID command for position
  geometry_msgs::Vector3 cmdMax;
  geometry_msgs::Vector3 cmdMin;
  
  ros::Time lastControlTime; 

  control_toolbox::Pid xCtrl;  ///< The PID controller for position x
  control_toolbox::Pid yCtrl;  ///< The PID controller for position y
  control_toolbox::Pid zCtrl;  ///< The PID controller for height
  
};

#endif
