//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_msgs/RadioControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#ifndef RADIO_CONTROL_MIXER_H
#define RADIO_CONTROL_MIXER_H

class RadioControlMixer
{
  public:
    RadioControlMixer();
  
  protected:
  
    void JoystickCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg);
    void ControllerCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg);
    void AutonomousCallback(const std_msgs::Int16::ConstPtr& autonomousMsg);
    
    void PublishRC();
  
    ros::NodeHandle nh, nh_priv;
    ros::Subscriber joystickSub; // control inputs from joystick, subscribed from /joystick_converter
    ros::Subscriber controllerSub; // control inputs from high-level controller (e.g., position controller), subscribed from /controller
    ros::Subscriber autonomousSub;
    ros::Publisher rcPub; //mixed control inputs, published to /draganfly_converter
    
    quadrotor_msgs::RadioControl lastJoystickMsg, lastControllerMsg;
    int autonomousMode;
};

#endif
