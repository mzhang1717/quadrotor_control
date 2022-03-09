#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher posePub, velocityPub;

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
    geometry_msgs::PoseStamped pose = *poseMsg;
    pose.pose.position.x = pose.pose.position.x  + 2.0;
    pose.pose.position.y = pose.pose.position.y  + 3.0;
    pose.header.frame_id = "px4flow";
    posePub.publish(pose);
};

void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
{
    geometry_msgs::TwistStamped velocity = *velocityMsg;
    velocity.header.frame_id = "px4flow";
    velocityPub.publish(velocity);
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_px4");
    
    ros::NodeHandle nh, nh_priv;
    
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("x8/output/pose", 1, PoseCallback);
    ros::Subscriber velocitySub = nh.subscribe<geometry_msgs::TwistStamped>("x8/output/velocity", 1, VelocityCallback);
    
    posePub = nh_priv.advertise<geometry_msgs::PoseStamped>("pose_px4", 1); /// publishes pose messages
    velocityPub = nh_priv.advertise<geometry_msgs::TwistStamped>("velocity_px4", 1);  /// publishes velocity messages
    
    ros::spin();
    
    return 0;
}
