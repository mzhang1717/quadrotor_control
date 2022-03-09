#include <ros/ros.h>
#include <state_estimation/ExtendedKalmanFilter.h>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ekf_node");
  ros::NodeHandle nh;
  
#if defined(CALIBRATION) && defined(FIXED_REL_WORLD)
  static_assert(false, "Cannot define both CALIBRATION and FIXED_REL_WORLD"); // should trigger compiler error
#endif
  
  // Create filter class
  ExtendedKalmanFilter filter_node;
  ros::spin();
  
  return 0;
}

