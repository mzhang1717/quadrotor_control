#include <quadrotor_input/SimpleStateMachine.h>

geometry_msgs::Point operator*(geometry_msgs::Point vec, double val)
{
  vec.x *= val;
  vec.y *= val;
  vec.z *= val;
  return vec;
}

geometry_msgs::Point operator+(geometry_msgs::Point vec1, geometry_msgs::Point vec2)
{
  vec1.x += vec2.x;
  vec1.y += vec2.y;
  vec1.z += vec2.z;
  return vec1;
}

SimpleStateMachine::SimpleStateMachine()
: nh_priv("~") 
{ 
  rcSub = nh.subscribe<quadrotor_msgs::RadioControl>("radio_control", 1, &SimpleStateMachine::RCCallback, this);
  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, &SimpleStateMachine::PoseCallback, this);
  autonomousPub = nh_priv.advertise<std_msgs::Bool>("autonomous_mode", 1);
  controllerClient = nh.serviceClient<quadrotor_input::CommandController>("controller_command");
  
  nh_priv.param<double>("home_x", finalPosition.x, 0.0);
  nh_priv.param<double>("home_y", finalPosition.y, 0.0);
  nh_priv.param<double>("home_z", finalPosition.z, 0.2);
  
  currAutonomous = false;
}

nav_msgs::Path SimpleStateMachine::GeneratePath(int numIntervals, double fracStep, double timePerInterval)
{
  nav_msgs::Path pathMsg;
  pathMsg.poses.resize(numIntervals+1);
  
  for(int i=0; i < numIntervals-1; ++i)
  {
    pathMsg.poses[i].header.stamp = ros::Time(0) + ros::Duration(i*timePerInterval);
    
    if(i == 0)
      pathMsg.poses[i].pose.position = (lastPose.pose.position + (finalPosition*-1)) * fracStep + finalPosition;
    else
      pathMsg.poses[i].pose.position = (pathMsg.poses[i-1].pose.position  + (finalPosition*-1)) * fracStep + finalPosition;
  }
  
  pathMsg.poses[numIntervals-1].header.stamp = ros::Time(0) + ros::Duration((numIntervals-1)*timePerInterval);
  pathMsg.poses[numIntervals-1].pose.position = finalPosition;
  
  pathMsg.poses[numIntervals].header.stamp = pathMsg.poses[numIntervals-1].header.stamp + ros::Duration(timePerInterval);
  pathMsg.poses[numIntervals].pose.position = finalPosition;
  pathMsg.poses[numIntervals].pose.position.z = -1;
  
  return pathMsg;
}

void SimpleStateMachine::RCCallback(const quadrotor_msgs::RadioControl::ConstPtr& rcMsg)
{
  bool desiredAutonomous = rcMsg->gear >= 0;
  bool goHome = rcMsg->gear == 1; // otherwise just hover
  
  if(desiredAutonomous && !currAutonomous)  // just turned autonomous
  {
    quadrotor_input::CommandController commandSrv;
    commandSrv.request.running = true;
    
    if(goHome)
      commandSrv.request.path = GeneratePath(10, 0.7, 1.0);
    else
    {
      nav_msgs::Path pathMsg;
      pathMsg.poses.resize(1);
      pathMsg.poses[0].header.stamp = ros::Time(0); 
      pathMsg.poses[0].pose.position = lastPose.pose.position;
      
      commandSrv.request.path = pathMsg;
    }
    
    if(controllerClient.call(commandSrv))
    {
      if(commandSrv.response.success)
      {
        ROS_INFO_STREAM("State machine switched external controller "<<controllerClient.getService()<<" on");
        currAutonomous = true;
      }
      else
      {
        ROS_ERROR_STREAM("External controller "<<controllerClient.getService()<<" responded but didn't switch on");
      }
    }
    else
    {
      ROS_ERROR_STREAM("External controller "<<controllerClient.getService()<<" didn't respond");
    }
  }
  else if(!desiredAutonomous && currAutonomous) // just turned manual
  {
    quadrotor_input::CommandController commandSrv;
    commandSrv.request.running = false;
    
    if(controllerClient.call(commandSrv))
    {
      if(commandSrv.response.success)
      {
        ROS_INFO_STREAM("State machine switched external controller "<<controllerClient.getService()<<" off");
        currAutonomous = false;
      }
      else
      {
        ROS_ERROR_STREAM("External controller "<<controllerClient.getService()<<" responded but didn't switch off");
      }
    }
    else
    {
      ROS_ERROR_STREAM("External controller "<<controllerClient.getService()<<" didn't respond");
    }
  }
    
  std_msgs::Bool msg;
  msg.data = currAutonomous;
  autonomousPub.publish(msg);
}

void SimpleStateMachine::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  lastPose = *poseMsg;
}
