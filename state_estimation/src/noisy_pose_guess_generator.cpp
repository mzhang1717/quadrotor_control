#include <ros/ros.h>
#include <state_estimation/Utility.h>
#include <state_estimation/YAMLParser.h>
#include <state_estimation/Sample.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <fstream>

double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

geometry_msgs::Pose SE3ToPoseMsg(TooN::SE3<> se3Transform)
{
  TooN::Vector<3> v3Pos = se3Transform.get_translation();
  TooN::Matrix<3> m3Rot = se3Transform.get_rotation().get_matrix();
  
  // Similar to PoseMsgToSE3(..), use Bullet 3x3 matrix and quaternion as intermediaries
  
  // Initialize 3x3 matrix directly from TooN 3-matrix
  tf::Matrix3x3 btMat(m3Rot(0,0),m3Rot(0,1),m3Rot(0,2),m3Rot(1,0),m3Rot(1,1),m3Rot(1,2),m3Rot(2,0),m3Rot(2,1),m3Rot(2,2));
  tf::Quaternion btQuat;
  btMat.getRotation(btQuat);
  
  geometry_msgs::Pose pose;

  // Fill in position directly
  pose.position.x = v3Pos[0];
  pose.position.y = v3Pos[1];
  pose.position.z = v3Pos[2];
  
  // Get quaternion values from Bullet quat
  pose.orientation.x = btQuat.x();
  pose.orientation.y = btQuat.y();
  pose.orientation.z = btQuat.z();
  pose.orientation.w = btQuat.w();
  
  return pose;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "noisy_pose_guess_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  
  std::string xyz, rpy;
  nh_priv.param<std::string>("xyz", xyz, "");
  nh_priv.param<std::string>("rpy", rpy, "");
  
  ROS_ASSERT(!xyz.empty() && !rpy.empty());
  
  TooN::Vector<3> v3xyz, v3rpy;
  
  std::stringstream ss;
  ss << xyz;
  ss >> v3xyz;
  ss.clear();
  ss.str("");
  ss << rpy;
  ss >> v3rpy;
  
  tf::Quaternion q;
  q.setRPY(deg2rad(v3rpy[0]), deg2rad(v3rpy[1]), deg2rad(v3rpy[2]));
  
  TooN::SO3<> so3Rot = util::QuatToSO3(q);
  TooN::Vector<3> v3Pos = v3xyz;
  
  double posNoise, rotNoise;
  nh_priv.param<double>("pos_noise", posNoise, 0);
  nh_priv.param<double>("rot_noise", rotNoise, 0);
  
  std::cout<<"sample: "<<Sample::uniform(-1.0, 1.0)<<std::endl;
  
  TooN::Vector<3> v3PosNoise = TooN::makeVector(posNoise*Sample::uniform(-1.0, 1.0), posNoise*Sample::uniform(-1.0, 1.0), posNoise*Sample::uniform(-1.0, 1.0));
  TooN::Vector<3> v3RotNoiseDir = TooN::makeVector(Sample::uniform(-1.0, 1.0), Sample::uniform(-1.0, 1.0), Sample::uniform(-1.0, 1.0));
  TooN::normalize(v3RotNoiseDir);
  
  double rotNoiseAngle = deg2rad(rotNoise) * Sample::uniform(-1.0, 1.0);
  TooN::Vector<3> v3RotNoise = v3RotNoiseDir * rotNoiseAngle;
  
  TooN::Vector<3> v3PosWithNoise = v3Pos + v3PosNoise;
  TooN::SO3<> so3RotWithNoise = so3Rot * TooN::SO3<>::exp(v3RotNoise);
  
  std::string outputFileName;
  nh_priv.param<std::string>("output_file", outputFileName, "guess.yaml");
  
  std::cout<<"Writing output to "<<outputFileName<<std::endl;
  std::cout<<"XYZ position "<<v3xyz<<" with noise "<<posNoise<<std::endl;
  std::cout<<"RPY orientation "<<v3rpy<<" with noise "<<rotNoise<<" (all degrees)"<<std::endl;
  
  std::ofstream outFile;
  outFile.open(outputFileName.c_str());
  if(!outFile.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<outputFileName<<"]");
    ros::shutdown();
    return -1;
  }
  
  outFile<<"---"<<std::endl;
  yaml::writePose(outFile, SE3ToPoseMsg(TooN::SE3<>(so3RotWithNoise, v3PosWithNoise)));

  outFile.close();
  
  return 0;
}
