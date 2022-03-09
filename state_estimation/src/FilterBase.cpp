#include <ros/ros.h>
#include <state_estimation/FilterBase.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <state_estimation/Utility.h>

FilterBase::FilterBase() 
: nh_private("~")
{
  std::string gv;
  this->nh_private.param<std::string>("gravity_vector", gv, "0 0 -9.81");
  std::stringstream ss;
  ss << gv;
  ss >> this->gravity_vector;
  this->gravity_magnitude = TooN::norm(this->gravity_vector);
  
  this->nh_private.param<bool>("fix_xy_output", this->fix_xy_output, false);
  
  this->nh_private.param<std::string>("world_frame", this->world_frame, "world");
  this->nh_private.param<std::string>("filter_frame", this->filter_frame, "filter");
  
  this->nh_private.param<bool>("imu_only_yaw", this->imu_only_yaw, false);
  
  this->imu_initialized = false;
  
  // advertise our estimation
  this->state_pub = nh_private.advertise<state_estimation::AircraftStateMsg>("state",1);
  this->pose_pub = nh_private.advertise<geometry_msgs::PoseStamped>("pose",1);
  this->velocity_pub = nh_private.advertise<geometry_msgs::TwistStamped>("velocity",1);
  
  this->imu_sub = nh.subscribe("imu", 1, &FilterBase::imuCallback, this);
  this->rel_pose_sub = nh.subscribe("rel_pose", 1, &FilterBase::relPoseCallback, this);
  this->px4flow_sub = nh.subscribe("px4flow", 1, &FilterBase::px4FlowCallback, this);
}

FilterBase::~FilterBase()
{
} 
  
void FilterBase::imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
  // Only use the acceleration and angular velocity from imu message, not 
  // the orientation
  
  ros::Time nowtime = imuMsg->header.stamp;
  
  if(!this->imu_initialized)
  {
    this->imu_initialized = true;
    this->last_predict = nowtime;
  }
  
  if(nowtime == this->last_predict)
    return;
  
  double dt = (nowtime - this->last_predict).toSec();
  this->last_predict = nowtime;
  
  TooN::Vector<3> accel_meas = TooN::makeVector(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);
  accel_meas *= this->gravity_magnitude;
  TooN::Vector<3> gyro_meas = TooN::makeVector(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
  
  double accel_cov = imuMsg->linear_acceleration_covariance[0];
  accel_cov *= this->gravity_magnitude;
  double gyro_cov = imuMsg->angular_velocity_covariance[0];
  double accel_bias_cov = imuMsg->linear_acceleration_covariance[1];
  accel_bias_cov *= this->gravity_magnitude;
  double gyro_bias_cov = imuMsg->angular_velocity_covariance[1];

  /*
  std::cout<<"Got imu callback dt: "<<dt<<std::endl;
  std::cout<<"accel_meas: "<<accel_meas<<std::endl;
  std::cout<<"accel_cov: "<<std::endl<<accel_cov<<std::endl;
  std::cout<<"gyro_meas: "<<gyro_meas<<std::endl;
  std::cout<<"gyro_cov: "<<std::endl<<gyro_cov<<std::endl;
  */
  
  imuCallbackInternal(accel_meas, accel_cov, accel_bias_cov, gyro_meas, gyro_cov, gyro_bias_cov, dt, this->gravity_vector, nowtime);
  
  // Start on magnetometer-based orientation data 
  tf::Quaternion q(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
  TooN::SO3<> rot = util::QuatToSO3(q); 
  TooN::Matrix<3> cov = TooN::Identity;
  //cov *= imuMsg->orientation_covariance[0];
  
  if(this->imu_only_yaw)
  {
	yawCallbackInternal(rot, cov, imuMsg->header.stamp);  
  }
  else
  {
    rotCallbackInternal(rot, cov, imuMsg->header.stamp);
  }
  
  state_estimation::AircraftStateMsg stateMsg;
  fillStateMessage(stateMsg);
  stateMsg.header.stamp = nowtime;
  stateMsg.header.frame_id = this->world_frame;
  state_pub.publish(stateMsg);
  
  geometry_msgs::PoseStamped poseOutMsg;
  poseOutMsg.pose.position = stateMsg.position;
  
  if(this->fix_xy_output)
  {
    poseOutMsg.pose.position.x = 0;
    poseOutMsg.pose.position.y = 0;
  }
  
  poseOutMsg.pose.orientation = stateMsg.rotation;
  poseOutMsg.header.stamp = nowtime;
  poseOutMsg.header.frame_id = this->world_frame;
  pose_pub.publish(poseOutMsg); 
  
  geometry_msgs::TwistStamped velocityMsg;
  velocityMsg.header.stamp = nowtime;
  velocityMsg.header.frame_id = this->world_frame;
  velocityMsg.twist.linear = stateMsg.velocity;
  velocityMsg.twist.angular = imuMsg->angular_velocity;
  velocity_pub.publish(velocityMsg);
  
  tf::Transform transform;
  tf::poseMsgToTF(poseOutMsg.pose, transform);
  br.sendTransform(tf::StampedTransform(transform, poseOutMsg.header.stamp, this->world_frame, this->filter_frame));

  //std::cout<<"Published kf_world to kf_pose, rel frame: "<<this->rel_pose_frame<<std::endl;

  if(!this->rel_pose_frame.empty())
  {
    //std::cout<<"Publishing transform from kf_world to "<<this->rel_pose_frame<<std::endl;
    transform.setOrigin( tf::Vector3(0,0,0) );
    transform.setRotation( tf::Quaternion(stateMsg.rot_r.x, stateMsg.rot_r.y, stateMsg.rot_r.z, stateMsg.rot_r.w) );
    br.sendTransform(tf::StampedTransform(transform, nowtime, this->world_frame, this->rel_pose_frame));
  }
}

void FilterBase::relPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
  if(!this->imu_initialized)
    return;
    
  this->rel_pose_frame = poseMsg->header.frame_id;
  TooN::Vector<3> rel_pos = TooN::makeVector(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, poseMsg->pose.pose.position.z);
  tf::Quaternion q(poseMsg->pose.pose.orientation.x, poseMsg->pose.pose.orientation.y, poseMsg->pose.pose.orientation.z, poseMsg->pose.pose.orientation.w);
  TooN::SO3<> rel_rot = util::QuatToSO3(q);
  TooN::Matrix<6> rel_cov = TooN::wrapMatrix<6,6>(&poseMsg->pose.covariance[0]);
  
  relPoseCallbackInternal(rel_pos, rel_rot, rel_cov, poseMsg->header.stamp);
}

void FilterBase::px4FlowCallback(const px_comm::OpticalFlow::ConstPtr& flowMsg)
{
  if(!this->imu_initialized)
    return;
    
  if(flowMsg->quality == 0)
  {
    ROS_WARN_STREAM("Zero quality px4flow message received, ignoring");
    return;
  }
    
  TooN::Vector<2> vel = TooN::makeVector(flowMsg->velocity_x, flowMsg->velocity_y);
  TooN::Matrix<2> cov = TooN::Identity;
  cov *= (50.0/flowMsg->quality); // hacky covariance matrix definition
  //cov *= (1.0/flowMsg->quality); // hacky covariance matrix definition
  
  bodyXYVelCallbackInternal(vel, cov, flowMsg->header.stamp);
  
  double distCov = 0.1;
  //double distCov = 0.05;
  
  groundDistanceCallbackInternal(flowMsg->ground_distance, distCov, flowMsg->header.stamp);
}

   
