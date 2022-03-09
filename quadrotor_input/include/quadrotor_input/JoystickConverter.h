//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <joy/Joy.h>
//#include <dynamic_reconfigure/server.h>
//#include <quadrotor_input/JoystickConverterConfig.h>
#include <quadrotor_msgs/RadioControl.h>

#ifndef JOYSTICK_CONVERTER_H
#define JOYSTICK_CONVERTER_H

/// Handles inputs from a joy node and translates them to DraganflyRadioControl message. Also togglest external controller on/off
class JoystickConverter
{
public:
  JoystickConverter();

private:

  /// Callback called when a joy message arrives. Main computation is done here so if you want to speed things up increase
  /// the rate of joy messages in the joy node launch params
  void joyCallback(const joy::Joy::ConstPtr& joy);
  
  /// Callback called when dynamic reconfigure changes a parameter
  //void reconfigureCallback(quadrotor_input::JoystickConverterConfig &config, uint32_t level);
  
  quadrotor_msgs::RadioControl joyToRC(const joy::Joy& joyMsg);
  
  double scaleThrottle(double throttle);
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  
  // Axes on the joystick that correspond to different channels
  int axisThrottle;
  int axisRoll;
  int axisPitch;
  int axisYaw;
  int axisFlap;
  int axisGear;
  
  // Channel value will be multiplied by 1 or -1 depending on user's desire to flip channel
  int signThrottle;
  int signRoll;
  int signPitch;
  int signYaw;
  int signFlap;
  int signGear;
  
  // Used to make the throttle more sensitive in a given region
  std::vector<double> throttlePolyCoeffs;

  // Publishes RadioControl messages
  ros::Publisher radioControlPub;
  
  // Subscribes to joy messages
  ros::Subscriber joySub;
  
  //dynamic_reconfigure::Server<quadrotor_input::JoystickConverterConfig> reconfigureServer;
};

#endif
