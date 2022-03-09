#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <state_estimation/PoseGenConfig.h>
#include <state_estimation/YAMLParser.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

tf::Transform transform;
tf::Transform transformInverse;
bool bTFInverse;

double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

void callback(state_estimation::PoseGenConfig &config, uint32_t level) 
{        
  transform.setOrigin(tf::Point(config.x, config.y, config.z));
  tf::Quaternion q;
  if(config.rot_type == 0) // euler
  {
    q.setEulerZYX(deg2rad(config.yaw), deg2rad(config.pitch), deg2rad(config.roll));
  }
  else  // quaternion
  {
    q = tf::Quaternion(config.qx, config.qy, config.qz, config.qw);
    q.normalize();
  }
  
  transform.setRotation(q);
  
  TooN::SO3<> so3Rot = util::QuatToSO3(transform.getRotation());
  TooN::SE3<> se3Pose(so3Rot, TooN::makeVector(config.x, config.y, config.z));
  TooN::SE3<> se3PoseInv = se3Pose.inverse();
  TooN::Vector<3> v3TransInv = se3PoseInv.get_translation();
  
  transformInverse.setRotation(util::SO3ToQuat(se3PoseInv.get_rotation()));
  transformInverse.setOrigin(tf::Point(v3TransInv[0], v3TransInv[1], v3TransInv[2]));
  
  bTFInverse = config.tf_inverse;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_generator");
  ros::NodeHandle nh_priv("~");
  
  std::string world_name, object_name;
  nh_priv.param<std::string>("world_name", world_name, "pose_gen_world");
  nh_priv.param<std::string>("object_name", object_name, "pose_gen_object");
  
  tf::TransformBroadcaster br;

  dynamic_reconfigure::Server<state_estimation::PoseGenConfig> server;
  server.setCallback(boost::bind(&callback, _1, _2));

  std::cerr<<"Run dynamic_reconfigure/reconfigure_gui to change parameters"<<std::endl;
  std::cerr<<"Broadcast frames are "<<world_name<<" and "<<object_name<<", view them in RViz"<<std::endl;
  std::cerr<<"I will print the final frame to stdout upon shutdown"<<std::endl;
  
  ros::Rate rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    
    if(bTFInverse)
      br.sendTransform(tf::StampedTransform(transformInverse, ros::Time::now(), object_name, world_name));
    else
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_name, object_name));
    
    rate.sleep();
  }
  
  yaml::writePose(std::cout, transform);
  
  TooN::SO3<> so3Rot = util::QuatToSO3(transform.getRotation());
  std::cout<<"Rotation matrix: "<<std::endl<<so3Rot<<std::endl;
  std::cout<<"Axis angle: "<<so3Rot.ln()<<std::endl;
  
  return 0;
}
