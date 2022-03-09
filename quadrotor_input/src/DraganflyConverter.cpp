#include <quadrotor_input/DraganflyConverter.h>
#include <quadrotor_msgs/DraganflyRadioControl.h>

DraganflyConverter::DraganflyConverter()
{ 
  rcSub = nh.subscribe<quadrotor_msgs::RadioControl>("input", 1, &DraganflyConverter::RCCallback, this);
  draganflyPub = nh.advertise<quadrotor_msgs::DraganflyRadioControl>("output", 1);
}

// converte RC control commands to int16 between -511 to +511.
void DraganflyConverter::RCCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg)
{
  quadrotor_msgs::DraganflyRadioControl draganflyMsg;
  
  draganflyMsg.header.stamp = ros::Time::now();
  draganflyMsg.throttle = roundf(511.0 * rcMsg->throttle);
  draganflyMsg.roll = roundf(511.0 * rcMsg->roll);
  draganflyMsg.pitch = roundf(511.0 * rcMsg->pitch);
  draganflyMsg.yaw = roundf(511.0 * rcMsg->yaw);
  draganflyMsg.ascent = 511 * rcMsg->flap;
  
  // It is important that some channels hit +/- 511 exactly, make sure that happens
  
  if(draganflyMsg.throttle < -500)
    draganflyMsg.throttle = -511;
    
  if(draganflyMsg.yaw < -500)
    draganflyMsg.yaw = -511;

  if(draganflyMsg.yaw > 500)
    draganflyMsg.yaw = 511;
  
  draganflyPub.publish(draganflyMsg);
}

