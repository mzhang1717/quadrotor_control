#include <ros/ros.h>
#include <state_estimation/AircraftStateMsg.h>
#include <state_estimation/YAMLParser.h>
#include <yaml-cpp/yaml.h>

bool done = false;
geometry_msgs::Pose calibPose;
geometry_msgs::Pose worldPose;

void stateCallback(const state_estimation::AircraftStateMsg::ConstPtr& stateMsg)
{
  calibPose.position = stateMsg->pos_c;
  calibPose.orientation = stateMsg->rot_c;
  worldPose.orientation = stateMsg->rot_r;
  done = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grab_calib_pose");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("state", 1, stateCallback);
  
  std::cerr<<"Subscribed to "<<sub.getTopic()<<", waiting for message"<<std::endl;
  std::cerr<<"I will print the received calibration pose to stdout upon completion"<<std::endl;
  
  ros::Rate rate(20);
  while(ros::ok() && !done)
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  if(!done)
  {
    std::cerr<<"Got nothing..."<<std::endl;
  }
  else
  {
    std::cerr<<"Got message!"<<std::endl;
    std::cerr<<"Pose of relative frame wrt IMU: "<<std::endl;
    yaml::writePose(std::cout, calibPose);
    std::cerr<<std::endl;
    std::cerr<<"Pose of relative world wrt inertial world: "<<std::endl;
    yaml::writePose(std::cout, worldPose);
    std::cerr<<std::endl;
  }
  
  return 0;
}
