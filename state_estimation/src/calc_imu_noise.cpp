#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <TooN/TooN.h>

std::queue<sensor_msgs::Imu> imuQueue;
unsigned int maxLength = 1000;
unsigned int minLength = 10;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
  imuQueue.push(*imuMsg);
  if(imuQueue.size() > maxLength)
  {
    imuQueue.pop();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calc_imu_noise");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("imu", 1, imuCallback);
  
  std::cerr<<"Subscribed to "<<sub.getTopic()<<", accumulating imu messages"<<std::endl;
  std::cerr<<"Upon completion I will calculate the accel and gyro noise using the accumulated imu messages (max "<<maxLength<<" messages)"<<std::endl;
  
  ros::Rate rate(1000);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  if(imuQueue.size() < minLength)
  {
    std::cerr<<"Accumulated "<<imuQueue.size()<<" messages, not enough to calculate noise, need at least "<<minLength<<std::endl;
    return -1;
  }
  
  TooN::Vector<3> meanAccel = TooN::Zeros;
  TooN::Vector<3> meanGyro = TooN::Zeros;
  
  std::vector<TooN::Vector<3> > accelVec;
  std::vector<TooN::Vector<3> > gyroVec;
  
  while(imuQueue.size() > 0)
  {
    sensor_msgs::Imu imuMsg = imuQueue.front();
    imuQueue.pop();
    
    accelVec.push_back(TooN::makeVector(imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z));
    meanAccel += accelVec.back();
    
    gyroVec.push_back(TooN::makeVector(imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z));
    meanGyro += gyroVec.back();
  }
  
  meanAccel /= accelVec.size();
  meanGyro /= gyroVec.size();
  
  std::cout<<"Number of measurements: "<<accelVec.size()<<std::endl;
  std::cout<<"Mean accel: "<<meanAccel<<std::endl;
  std::cout<<"Mean gyro: "<<meanGyro<<std::endl;
  
  TooN::Matrix<3> accelCov = TooN::Zeros;
  TooN::Matrix<3> gyroCov = TooN::Zeros;
  
  for(unsigned i=0; i < accelVec.size(); ++i)
  {
    accelCov += (accelVec[i] - meanAccel).as_col() * (accelVec[i] - meanAccel).as_row();
    gyroCov += (gyroVec[i] - meanGyro).as_col() * (gyroVec[i] - meanGyro).as_row();
  }
  
  accelCov /= (accelVec.size() - 1);
  gyroCov /= (gyroVec.size() - 1);
  
  std::cout<<"Accelerometer covariance: "<<std::endl<<accelCov<<std::endl;
  std::cout<<"Gyroscope covariance: "<<std::endl<<gyroCov<<std::endl;
  
  return 0;
}
