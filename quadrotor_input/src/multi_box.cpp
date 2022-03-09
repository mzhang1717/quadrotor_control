#include <ros/ros.h>
#include <quadrotor_input/MultiBox.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_box");
  MultiBox mBox;

  ros::spin();
}
