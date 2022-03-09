//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_msgs/RadioControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <quadrotor_input/CommandController.h>
#include <nav_msgs/Path.h>

#ifndef SIMPLE_STATE_MACHINE_H
#define SIMPLE_STATE_MACHINE_H

class SimpleStateMachine
{
  public:
    SimpleStateMachine();
  
  protected:
  
    nav_msgs::Path GeneratePath(int numIntervals, double fracStep, double timePerInterval);
  
    void RCCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg);
    
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
  
    ros::NodeHandle nh, nh_priv;
    ros::Subscriber rcSub;
    ros::Subscriber poseSub;
    ros::Publisher autonomousPub;
    ros::ServiceClient controllerClient;
    
    bool currAutonomous;
    geometry_msgs::PoseStamped lastPose;
    geometry_msgs::Point finalPosition;
};

#endif
