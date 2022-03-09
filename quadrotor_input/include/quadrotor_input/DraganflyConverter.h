//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_msgs/RadioControl.h>

#ifndef DRAGANFLY_CONVERTER_H
#define DRAGANFLY_CONVERTER_H

class DraganflyConverter
{
  public:
    DraganflyConverter();
  
  protected:
  
    void RCCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg);
    
    ros::NodeHandle nh;
    ros::Subscriber rcSub;
    ros::Publisher draganflyPub;
};

#endif
