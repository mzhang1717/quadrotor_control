#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_input/SelectStateSource.h>

#ifndef MULTI_BOX_H
#define MULTI_BOX_H

//static const NUMBER_OF_SOURCE = 2;

class MultiBox
{
public:
    MultiBox();

private:
    std::vector< std::string > m_strPoseTopicNames;
    std::vector< std::string > m_strVelocityTopicNames;

    std::string m_strPoseTopicSelected;
    std::string m_strVelocityTopicSelected;

    ros::NodeHandle m_nh, m_nh_priv;
    ros::ServiceServer m_SelectTopicService;  ///< Service to select the pose and velocity topics from multiple sources
    
    std::vector< ros::Subscriber > m_vPoseSub;
    std::vector< ros::Subscriber > m_vVelocitySub;
    
    ros::Publisher m_posePub;  ///< Publishes pose messages
    ros::Publisher m_velocityPub; /// publishes velocity messages
    
    bool SelectTopicServiceCallback(quadrotor_input::SelectStateSource::Request &req, quadrotor_input::SelectStateSource::Response &res);
    
    // Current pose (position & orientation)
    //geometry_msgs::PoseStamped m_MsgPoseStamped_A, m_MsgPoseStamped_B;
    //geometry_msgs::TwistStamped m_TwistStamped_A, m_TwistStamped_B; // velocity in body frame
    
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, std::string topicSource);
    void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg, std::string topicSource);
    
    /// Callback called by the pose subscriber (if subscribed)
    //void PoseCallback_A(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
    //void VelocityCallback_A(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg);
    
    /// Callback called by the pose subscriber (if subscribed)
    //void PoseCallback_B(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
    //void VelocityCallback_B(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg);    
    
    void PublishPose(const geometry_msgs::PoseStamped poseMsg);
    void PublishVelocity(const geometry_msgs::TwistStamped velocityMsg);
};

#endif
