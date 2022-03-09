#include <ros/ros.h>
#include <state_estimation/SimpleVelocityFilter.h>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "simple_velocity_filter");
  ros::NodeHandle nh;
  
  // Create filter class
  SimpleVelocityFilter filter_node;
  ros::spin();
  
  return 0;
}

