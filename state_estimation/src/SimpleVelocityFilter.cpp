#include <ros/ros.h>
#include <state_estimation/SimpleVelocityFilter.h>
#include <geometry_msgs/TwistStamped.h>

SimpleVelocityFilter::SimpleVelocityFilter() 
: nh_private("~")
{
  // advertise our estimation
  this->velocity_pub = nh_private.advertise<geometry_msgs::TwistStamped>("velocity",1);
  
  this->rel_pose_sub = nh.subscribe("rel_pose", 1, &SimpleVelocityFilter::relPoseCallback, this);
  this->rel_pose_cov_sub = nh.subscribe("rel_pose_cov", 1, &SimpleVelocityFilter::relPoseCovCallback, this);
}

SimpleVelocityFilter::~SimpleVelocityFilter()
{
} 
  
void SimpleVelocityFilter::relPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  static bool first = true;
  if(first)
  {
    first = false;
    lastPoseMsg = poseMsg->pose;
    lastTime = poseMsg->header.stamp;
    return;
  }
  
  if(poseMsg->header.stamp == lastTime)
    return;
  
  ros::Duration dt = poseMsg->header.stamp - lastTime;
  
  geometry_msgs::TwistStamped velocityMsg;
  velocityMsg.header.stamp = poseMsg->header.stamp;
  velocityMsg.header.frame_id = "kf_world";
  velocityMsg.twist.linear.x = (poseMsg->pose.position.x - lastPoseMsg.position.x) / dt.toSec();
  velocityMsg.twist.linear.y = (poseMsg->pose.position.y - lastPoseMsg.position.y) / dt.toSec();
  velocityMsg.twist.linear.z = (poseMsg->pose.position.z - lastPoseMsg.position.z) / dt.toSec();
  velocity_pub.publish(velocityMsg);
  
  lastPoseMsg = poseMsg->pose;
  lastTime = poseMsg->header.stamp;
}

void SimpleVelocityFilter::relPoseCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
  static bool first = true;
  if(first)
  {
    first = false;
    lastPoseMsg = poseMsg->pose.pose;
    lastTime = poseMsg->header.stamp;
    return;
  }
  
  if(poseMsg->header.stamp == lastTime)
    return;
  
  ros::Duration dt = poseMsg->header.stamp - lastTime;
  
  geometry_msgs::TwistStamped velocityMsg;
  velocityMsg.header.stamp = poseMsg->header.stamp;
  velocityMsg.header.frame_id = "kf_world";
  velocityMsg.twist.linear.x = (poseMsg->pose.pose.position.x - lastPoseMsg.position.x) / dt.toSec();
  velocityMsg.twist.linear.y = (poseMsg->pose.pose.position.y - lastPoseMsg.position.y) / dt.toSec();
  velocityMsg.twist.linear.z = (poseMsg->pose.pose.position.z - lastPoseMsg.position.z) / dt.toSec();
  velocity_pub.publish(velocityMsg);
  
  lastPoseMsg = poseMsg->pose.pose;
  lastTime = poseMsg->header.stamp;
}
   
