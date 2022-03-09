#include <quadrotor_input/MultiBox.h>
#include "boost/bind.hpp"
#include <assert.h>

MultiBox::MultiBox()
: m_nh_priv("~") 
{
    m_strPoseTopicNames.clear();
    m_strVelocityTopicNames.clear();
    
    m_nh_priv.getParam("pose_topic_list", m_strPoseTopicNames);
    m_nh_priv.getParam("velocity_topic_list", m_strVelocityTopicNames);
    
    int nPoseTopics = m_strPoseTopicNames.size();
    int nVelocityTopics = m_strVelocityTopicNames.size();
    
    assert(nPoseTopics > 0 && nVelocityTopics > 0);
    
    m_vPoseSub.resize(nPoseTopics);
    for (int i = 0; i < nPoseTopics; i++)
    {
        m_vPoseSub[i] = m_nh.subscribe<geometry_msgs::PoseStamped>(m_strPoseTopicNames[i], 1, boost::bind(&MultiBox::PoseCallback, this, _1, m_strPoseTopicNames[i]));
    }
    
    m_vVelocitySub.resize(nVelocityTopics);
    for (int i = 0; i < nVelocityTopics; i++)
    {
        m_vVelocitySub[i] = m_nh.subscribe<geometry_msgs::TwistStamped>(m_strVelocityTopicNames[i], 1, boost::bind(&MultiBox::VelocityCallback, this, _1, m_strVelocityTopicNames[i]));
    }
    //poseSub_A = nh.subscribe<geometry_msgs::PoseStamped>("pose_px4", 10, &PositionController::PoseCallback_A, this);
    //velocitySub_A = nh.subscribe<geometry_msgs::TwistStamped>("velocity_px4", 10, &PositionController::VelocityCallback_A, this);
    
    //poseSub_B = nh.subscribe<geometry_msgs::PoseStamped>("pose_mcptam", 10, &PositionController::PoseCallback_B, this);
    //velocitySub_B = nh.subscribe<geometry_msgs::TwistStamped>("velocity_mcptam", 10, &PositionController::VelocityCallback_B, this);
    
    m_posePub = m_nh_priv.advertise<geometry_msgs::PoseStamped>("pose", 1);
    m_velocityPub = m_nh_priv.advertise<geometry_msgs::TwistStamped>("velocity", 1);
    
    m_SelectTopicService = m_nh_priv.advertiseService("select_topic", &MultiBox::SelectTopicServiceCallback, this);
}

void MultiBox::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, std::string topicSource)
{
    if (m_strPoseTopicSelected == topicSource)
    {
        PublishPose(*poseMsg);
    }
}

void MultiBox::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg, std::string topicSource)
{
    if (m_strVelocityTopicSelected == topicSource)
    {
        PublishVelocity(*velocityMsg);
    }
}

////px4flow
//void MultiBox::PoseCallback_A(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
//{
    ////m_MsgPoseStamped_A = poseMsg;
    
    //if (m_TopicSource == PX4FLOW)
    //{
        //PublishPose(poseMsg);
    //}
//}

////px4flow
//void MultiBox::VelocityCallback_A(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
//{
    ////m_TwistStamped_A = velocityMsg;
    
    //if (m_TopicSource == PX4FLOW)
    //{
        //PublishVelocity(velocityMsg);
    //}
//}

////mcptam
//void MultiBox::PoseCallback_B(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
//{
    ////m_MsgPoseStamped_B = poseMsg;
    
    //if (m_TopicSource == MCPTAM)
    //{
        //PublishPose(poseMsg);
    //}
//}

////mcptam
//void MultiBox::VelocityCallback_B(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
//{
    ////m_TwistStamped_A = velocityMsg;
    
    //if (m_TopicSource == MCPTAM)
    //{
        //PublishVelocity(velocityMsg);
    //}
//}

void MultiBox::PublishPose(const geometry_msgs::PoseStamped poseMsg)
{
    m_posePub.publish(poseMsg);
}

void MultiBox::PublishVelocity(const geometry_msgs::TwistStamped velocityMsg)
{
    m_velocityPub.publish(velocityMsg);
}

bool MultiBox::SelectTopicServiceCallback(quadrotor_input::SelectStateSource::Request &req, quadrotor_input::SelectStateSource::Response &res)
{
    m_strPoseTopicSelected = req.strPoseTopicSelected;
    m_strVelocityTopicSelected = req.strVelocityTopicSelected;
 
    std::vector< std::string >::iterator it, iter;
    
    it = find(m_strPoseTopicNames.begin(), m_strPoseTopicNames.end(), m_strPoseTopicSelected);
    iter = find(m_strVelocityTopicNames.begin(), m_strVelocityTopicNames.end(), m_strVelocityTopicSelected);
    
    if (it != m_strPoseTopicNames.end() && iter != m_strVelocityTopicNames.end()) //all selected topics are in the list
    {
        ROS_INFO_STREAM("Switched to: " << m_strPoseTopicSelected << " & " << m_strVelocityTopicSelected);
        res.bSuccess =  true;
        return true;
    }
    else
    {
        ROS_INFO("Failed to change the source of state estimation...");
        res.bSuccess =  false;
        return false;
    }
}
