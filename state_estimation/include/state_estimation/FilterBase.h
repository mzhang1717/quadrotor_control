#ifndef FILTER_BASE_H
#define FILTER_BASE_H

#include <ros/ros.h>
#include <state_estimation/AircraftStateMsg.h>
#include <TooN/TooN.h>
#include <TooN/so3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <px_comm/OpticalFlow.h>

class FilterBase
{
  public:
    FilterBase();
    ~FilterBase();
    
  private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg);
    void relPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
    void px4FlowCallback(const px_comm::OpticalFlow::ConstPtr& flowMsg);
    
    // These functions MUST be overridden in derived class
    virtual void fillStateMessage(state_estimation::AircraftStateMsg &state_msg) = 0;
    virtual void imuCallbackInternal(TooN::Vector<3> accel_meas, double accel_cov, double accel_bias_cov, TooN::Vector<3> gyro_meas, double gyro_cov, double gyro_bias_cov, double dt, TooN::Vector<3> gravity, ros::Time stamp) = 0;
    
    // These functions can be overridden in derived class if we expect to receive the corresponding message type
    virtual void rotCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp){ ROS_ERROR("In rotCallbackInternal in FilterBase, need to override this!"); };
    virtual void yawCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp){ ROS_ERROR("In yawCallbackInternal in FilterBase, need to override this!"); };
    virtual void relPoseCallbackInternal(TooN::Vector<3> rel_pos, TooN::SO3<> rel_rot, TooN::Matrix<6> rel_cov, ros::Time stamp){ ROS_ERROR("In relPoseCallbackInternal in FilterBase, need to override this!"); };
    virtual void bodyXYVelCallbackInternal(TooN::Vector<2> vel, TooN::Matrix<2> cov, ros::Time stamp){ ROS_ERROR("In bodyXYVelCallbackInternal in FilterBase, need to override this!"); };
    virtual void groundDistanceCallbackInternal(double distance, double cov, ros::Time stamp){ ROS_ERROR("In groundDistanceCallbackInternal in FilterBase, need to override this!"); };
    
    ros::Publisher state_pub, pose_pub, velocity_pub;  // pose_pub for rviz
    ros::Subscriber imu_sub, rel_pose_sub, px4flow_sub;
    
    tf::TransformBroadcaster br;

    TooN::Vector<3> gravity_vector;
    double gravity_magnitude;
    
    bool imu_only_yaw;
   
    std::string world_frame;
    std::string filter_frame;
    std::string rel_pose_frame;
    bool imu_initialized;
    bool fix_xy_output;
    
    ros::Time last_predict;
    
  protected:
    ros::NodeHandle nh, nh_private;
    
};


#endif

