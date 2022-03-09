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
#include <std_msgs/Int16.h>

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
  
  double CalcBatteryVoltage();
    
  double CalcBaseThrottle(double voltage);
  
  double CalcScale(double baseThrottle);
  
  geometry_msgs::Vector3 CalcErrorDerivative();
  
  void CalcErrorHistory();
  
  geometry_msgs::Pose GetDesiredPose();
  
  ros::Subscriber poseSub;
  ros::Subscriber velocitySub;
  ros::Subscriber batterySub;
  
  // Debug publisher
  ros::Publisher positionDebugPub; 
  ros::Publisher controlWarnPub; // publish warning information (delays in position/battery topics)
  ros::Publisher wpIndexPub; // publish the index of the current waypoint
  
  boost::circular_buffer<std::pair<ros::Time, geometry_msgs::Vector3> > errorHistory;
  boost::circular_buffer<double> voltageHistory;
  
  // Current pose (position & orientation)
  geometry_msgs::Pose currentPose;
  
  geometry_msgs::Vector3 currentVelocity; // velocity in body frame
  geometry_msgs::Vector3 currentVelocity_world; // velocity in inertial frame
  geometry_msgs::Vector3 currentEulerRPY; // Euler angle (roll, pitch, yaw)
  
  // Desired position and yaw angle
  geometry_msgs::Point desiredPosition;
  double desiredYaw;
  
  // Time stamp of variables
  ros::Time lastPositionTime;
  ros::Time lastVelocityTime;
  ros::Time lastBatteryTime;
  
  ros::Duration positionTimeBuffer;
  ros::Duration batteryTimeBuffer;
  ros::Duration velocityTimeBuffer;
  
  // batery voltage-throttle curve
  double voltageSetpointStart;
  double voltageSetpointEnd;
  double baseThrottleSetpointStart;
  double baseThrottleSetpointEnd;
  
  // vehicle mass
  double mass;
  
  bool xyActive;  // for debugging the xy control
  
  dynamic_reconfigure::Server<quadrotor_input::PositionControllerConfig> reconfigureServer;
  
  geometry_msgs::Vector3 pidLimit;  ///< Limit of allowed PID command for position
  geometry_msgs::Vector3 cmdMax;
  geometry_msgs::Vector3 cmdMin;
  
  geometry_msgs::Vector3 pidLimit_v;  ///< Limit of allowed PID command for speed
  geometry_msgs::Vector3 cmdMax_v;
  geometry_msgs::Vector3 cmdMin_v;
  
  double pidLimit_yaw;  ///< Limit of allowed PID command for yaw
  double cmdMax_yaw;
  double cmdMin_yaw;  
  
  // control tolance of position and yaw error
  double tolPosition;
  double tolYaw;
  
  // control commands convertion
  double scaleRP; // roll/pitch angle scale (1 ~ 511 ~ 34 deg)
  double convertRP; // = 180/pi/34.7
  
  // control commands limit
  double xyspeedLimit;
  
  double zspeedLimit;
  double ascendingSpeedMax;  
  double descendingSpeedMax;  
  double touchdownSpeedMax; 
  
  double pitchLimit;
  double rollLimit;
  double yawrateLimit;
  double throttleHighLimit;
  double throttleLowLimit;
  

  ros::Time lastControlTime; 
  
  control_toolbox::Pid pCtrl;  /// PID controller for horizontal distance
  
  //control_toolbox::Pid xCtrl;  ///< The PID controller for position x (in inertial frame)
  //control_toolbox::Pid yCtrl;  ///< The PID controller for position y (in inertial frame)
  control_toolbox::Pid zCtrl;  ///< The PID controller for height (in inertial frame)
  
  control_toolbox::Pid vxCtrl;  ///< The PID controller for speed x_dot (in inertial frame)
  control_toolbox::Pid vyCtrl;  ///< The PID controller for speed y_dot (in inertial frame)
  control_toolbox::Pid vzCtrl;  ///< The PID controller for vertical speed h_dot (in inertial frame)
  
  control_toolbox::Pid yawCtrl; ///< The PID controller for yaw angle
  
/*  
  //////////////////////////////////////////
  double Tau_Controller(double t, double t_f, double altitude,  double speed, double k, ros::Duration step);
  control_toolbox::Pid tauCtrl;
  double t_i;
  double time_elapsed;
  double tau;
  double tau_ref;
  double error;
  
  double tau_p;
  double tau_i;
  double tau_d;
  double tau_f;
  double tau_k;
  /////////////////////////////////////////
 */
 
 
    /////////////Tau Controller - Ben///////////
    //int sign(double x); //signum function - not built into C++
    //double tauRefX(double t_elapsed); //tau reference function in horizontal gap
    //double tauRefZ(double t_elapsed); //tau reference function in vertical gap
    //void updateGains(double t_elapsed);
    //double desiredHorizontalAcceleration_tau(double t_elapsed, ros::Duration step);
    //double desiredVerticalAcceleration_tau(double t_elapsed, ros::Duration step);

    //control_toolbox::Pid tauCtrlX;
    //control_toolbox::Pid tauCtrlZ;
    
    ////Set all constants and parameters
    //double kz; //z-gap tau constant
    //double kp; //phi-gap tau constant
    
    //double tf; //desired time to complete trajectory
    //double t_i; //for timers
    //double t_offset; //start timer at 0.1s for reference Tau generation
    //double t_elapsed; //for timers
    
    //// position to land from - used for reference generation:
    //double x_0, z_0, phi_0;
    //double vertical_offset; // height to go to
    
    //double horizontal_offset; // horizontal offset (from start)
    //double horizontalStartPos, horizontalFinishPos;

    //double holdPosY; // for backup controller

    //double errorX, errorZ;
    //double tauX, tauZ;
    //double velX_fixed, velZ_fixed;

    
    ////PID gains for Tau controllers and pitch angle controller
    //double kp_x, ki_x, kd_x, kp_z, ki_z, kd_z;
    ////double kp_theta;
    ////double ki_theta;
    ////double kd_theta;
    
    //// double tau_x_gains_scaling, tau_z_gains_scaling; // add these for dynamic reconfigure
    //double kp_x_scale, ki_x_scale, kd_x_scale, kp_z_scale, ki_z_scale, kd_z_scale;

    ////saturation limits for thrust signal
    //double xSaturationLimit, zSaturationLimit;
    
    //// moving average of hover throttle value
    //double baseThrottleAverage;
    
    //bool tau_enabled;
    //bool vertical_tau_only;

    //bool this_tau;
    //bool last_tau;
    
    //bool backupTriggered;
    ///////////////////////////////////////////
  
};

#endif
