#include <ros/ros.h>
#include <state_estimation/AircraftStateMsg.h>
#include <TooN/TooN.h>
#include <fstream>

std::ofstream outFile;
bool angle;

void stateCallback(const state_estimation::AircraftStateMsg::ConstPtr& stateMsg)
{	
  if(stateMsg->pos_c.x == 0)
  {
    return;
  }
  
  TooN::Matrix<24> cov = TooN::wrapMatrix<24,24>(&stateMsg->covariance[0]);
  double posCovNorm = TooN::norm_fro(cov.slice<18,18,3,3>());
  double rotCovNorm = TooN::norm_fro(cov.slice<21,21,3,3>());
		
  outFile << stateMsg->header.stamp << ", " << stateMsg->pos_c.x << ", " << stateMsg->pos_c.y << ", " << stateMsg->pos_c.z << ", ";
  
  if(angle)
  {
    outFile << 2*acos(stateMsg->rot_c.w) << ", ";
  }
  else
  {
    outFile << stateMsg->rot_c.x << ", " << stateMsg->rot_c.y << ", " << stateMsg->rot_c.z << ", " << stateMsg->rot_c.w << ", ";
  }
  
  outFile << posCovNorm << ", " << rotCovNorm << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "write_calib_pose");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string outputFileName;
  nh_priv.param<std::string>("output_file", outputFileName, "calib_pose.csv");
  
  outFile.open(outputFileName.c_str());
  if(!outFile.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<outputFileName<<"]");
    ros::shutdown();
    return -1;
  }
  
  nh_priv.param<bool>("angle", angle, true);
  
  outFile << "time, x, y, z, ";
  if(angle)
  {
	  outFile <<"angle, ";
  }
  else
  {
	  outFile <<"qx, qy, qz, qw, ";
  }
  
  outFile << "pos_cov_norm, rot_cov_norm"<<std::endl;
  
  ros::Subscriber sub = nh.subscribe("state", 1, stateCallback);
  
  std::cerr<<"Subscribed to "<<sub.getTopic()<<", waiting for messages"<<std::endl;
  std::cerr<<"Writing output to "<<outputFileName<<std::endl;
  
  ros::spin();

  outFile.close();
  
  return 0;
}
