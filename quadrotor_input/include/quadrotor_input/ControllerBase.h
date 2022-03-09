//=========================================================================================
//
// Copyright 2013 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <ros/ros.h>
#include <quadrotor_msgs/RadioControl.h>
#include <quadrotor_msgs/ControllerError.h>
#include <quadrotor_input/CommandController.h>
#include <boost/thread/mutex.hpp>
#include "std_msgs/Bool.h"

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

/// Base class for all controllers that interface with DraganflyJoystick
class ControllerBase
{
public:
  ControllerBase();
  
  /// Called by main program, runs until shutdown
  void Run();  

protected:

  /// Override this to create your own initialize function, called when controller turns on
  virtual bool Init(){ return true; }
  
  /// Override this to create your own finishing function, called when controller turns off
  virtual bool Fini(){ return true; }
  
  /// Pure virtual function, must be overridden by a child class to implement the actual computation stuff
  virtual void Compute(quadrotor_msgs::RadioControl& rcMsg, quadrotor_msgs::ControllerError& errorMsg) = 0;
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  
  double rate;  ///< Loop rate
  
  enum ControllerStatus{INACTIVE, ACTIVE} status;
  enum ControllerChannels{THROTTLE, ROLL, PITCH, YAW};
  std::set<ControllerChannels> controllerChannels;  ///< Channels that are being controller, child class needs to add to this set in its constructor!
  
  unsigned int waypointIndex; /// index of next waypoint to follow
  
  nav_msgs::Path currentPath;
  std::vector<double> newGains;
  boost::mutex commandMutex;
  
  bool controller_status; //on/off
    
private:

  /// Callback called by notify service, used to switch controller on/off
  bool CommandServiceCallback(quadrotor_input::CommandController::Request &req, quadrotor_input::CommandController::Response &res);
  
  void SetCurrentPath(const nav_msgs::Path& path);

  ros::Publisher radioControlPub;  ///< Publishes RadioControl messages
  ros::Publisher errorPub;
  ros::ServiceServer commandService;  ///< Service to turn controller on/off and supply a path
  
  ros::Publisher controllerStatusPub; /// publishes the status of controller on/off
  
};

#endif
