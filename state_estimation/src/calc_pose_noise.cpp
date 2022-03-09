#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <state_estimation/Utility.h>
#include <queue>

std::queue<geometry_msgs::PoseStamped> poseQueue;
unsigned int maxLength = 1000;
unsigned int minLength = 10;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  poseQueue.push(*poseMsg);
  if(poseQueue.size() > maxLength)
  {
    poseQueue.pop();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calc_pose_noise");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("pose", 1, poseCallback);
  
  std::cerr<<"Subscribed to "<<sub.getTopic()<<", accumulating pose messages"<<std::endl;
  std::cerr<<"Upon completion I will calculate the position and rotation noise using the accumulated poses (max "<<maxLength<<" poses)"<<std::endl;
  
  ros::Rate rate(1000);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  if(poseQueue.size() < minLength)
  {
    std::cerr<<"Accumulated "<<poseQueue.size()<<" poses, not enough to calculate noise, need at least "<<minLength<<std::endl;
    return -1;
  }
  
  TooN::Vector<3> meanPos = TooN::Zeros;
  TooN::Vector<3> meanRot = TooN::Zeros;
  
  std::vector<TooN::Vector<3> > posVec;
  std::vector<TooN::Vector<3> > rotVec;
  
  while(poseQueue.size() > 0)
  {
    geometry_msgs::PoseStamped poseMsg = poseQueue.front();
    poseQueue.pop();
    
    posVec.push_back(TooN::makeVector(poseMsg.pose.position.x, poseMsg.pose.position.y, poseMsg.pose.position.z));
    meanPos += posVec.back();
    
    tf::Quaternion q(poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);
    rotVec.push_back((util::QuatToSO3(q)).ln());
    meanRot += rotVec.back();
  }
  
  meanPos /= posVec.size();
  meanRot /= rotVec.size();
  
  std::cout<<"Number of poses: "<<posVec.size()<<std::endl;
  std::cout<<"Mean pos: "<<meanPos<<std::endl;
  std::cout<<"Mean rot: "<<std::endl<<TooN::SO3<>::exp(meanRot)<<std::endl;
  
  TooN::Matrix<3> posCov = TooN::Zeros;
  TooN::Matrix<3> rotCov = TooN::Zeros;
  
  for(unsigned i=0; i < posVec.size(); ++i)
  {
    posCov += (posVec[i] - meanPos).as_col() * (posVec[i] - meanPos).as_row();
    rotCov += (rotVec[i] - meanRot).as_col() * (rotVec[i] - meanRot).as_row();
  }
  
  posCov /= (posVec.size() - 1);
  rotCov /= (rotVec.size() - 1);
  
  std::cout<<"Position covariance: "<<std::endl<<posCov<<std::endl;
  std::cout<<"Rotation covariance: "<<std::endl<<rotCov<<std::endl;
  
  return 0;
}
