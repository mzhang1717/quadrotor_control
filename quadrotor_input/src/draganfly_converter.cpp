#include <ros/ros.h>
#include <quadrotor_input/DraganflyConverter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draganfly_converter");
  DraganflyConverter dc;

  ros::spin();
}



