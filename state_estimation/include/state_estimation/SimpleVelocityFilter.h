#ifndef SIMPLE_VELOCITY_FILTER_H
#define SIMPLE_VELOCITY_FILTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

class SimpleVelocityFilter
{
  public:
    SimpleVelocityFilter();
    ~SimpleVelocityFilter();
    
  private:
    void relPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
    void relPoseCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
    
    ros::Publisher velocity_pub;
    ros::Subscriber rel_pose_cov_sub;
    ros::Subscriber rel_pose_sub;
    
    ros::Time lastTime;
    geometry_msgs::Pose lastPoseMsg;
    
  protected:
    ros::NodeHandle nh, nh_private;
    
};


#endif

