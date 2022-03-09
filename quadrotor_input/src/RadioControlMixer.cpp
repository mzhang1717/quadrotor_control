#include <quadrotor_input/RadioControlMixer.h>

RadioControlMixer::RadioControlMixer()
: nh_priv("~") 
{ 
  joystickSub = nh.subscribe<quadrotor_msgs::RadioControl>("joystick", 1, &RadioControlMixer::JoystickCallback, this);
  controllerSub = nh.subscribe<quadrotor_msgs::RadioControl>("controller", 1, &RadioControlMixer::ControllerCallback, this);
  autonomousSub = nh.subscribe<std_msgs::Int16>("autonomous", 1, &RadioControlMixer::AutonomousCallback, this);
  
  rcPub = nh_priv.advertise<quadrotor_msgs::RadioControl>("radio_control", 1);
  autonomousMode = 0;
}

void RadioControlMixer::JoystickCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg)
{
  lastJoystickMsg = *rcMsg;
  PublishRC();
}

void RadioControlMixer::ControllerCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg)
{
  lastControllerMsg = *rcMsg;
  // Don't publish RC here, since we want to STOP publishing anything
  // if we don't hear from the joystick! This means some kind of severe
  // communication error, and we want the aircraft to enter its failsafe
  // mode when it doesn't receive control messages fast enough
  //PublishRC();
}

void RadioControlMixer::AutonomousCallback(const std_msgs::Int16::ConstPtr& autonomousMsg)
{
  autonomousMode = autonomousMsg->data;
}

void RadioControlMixer::PublishRC()
{
  ros::Time nowTime = ros::Time::now();
  quadrotor_msgs::RadioControl rcMsg;
  
  // Last failsafe to make sure that we are truly in autonomous mode,
  // directly check the "gear" switch here as well
  bool isAuto = autonomousMode && (lastJoystickMsg.gear > -1);
  
  if(isAuto && lastControllerMsg.header.stamp - nowTime < ros::Duration(1.0))  // take the controller message
  {
    rcMsg = lastControllerMsg;
    rcMsg.header.stamp = nowTime;
    
    // Look for any illegal values in the controller message and replace them with the values from the joystick
    if(rcMsg.throttle > 1.0 || rcMsg.throttle < -1.0)
    rcMsg.throttle = lastJoystickMsg.throttle;
    
    if(rcMsg.roll > 1.0 || rcMsg.roll < -1.0)
    rcMsg.roll = lastJoystickMsg.roll;
    
    if(rcMsg.pitch > 1.0 || rcMsg.pitch < -1.0)
    rcMsg.pitch = lastJoystickMsg.pitch;
    
    if(rcMsg.yaw > 1.0 || rcMsg.yaw < -1.0)
    rcMsg.yaw = lastJoystickMsg.yaw;
    
    // for emergency only, shutting off engine
    if(lastJoystickMsg.throttle < -0.9)
    rcMsg.throttle = lastJoystickMsg.throttle;
    
    if (autonomousMode == 2) // idle mode, set throttle below 0.1
    {
    rcMsg.throttle = std::min(lastJoystickMsg.throttle, 0.1);
    }

  }
  else  // manual mode, take the joystick message
  {
    rcMsg = lastJoystickMsg;
    rcMsg.header.stamp = nowTime;
  }
  
  rcPub.publish(rcMsg);
}
