#include <ros/ros.h>
#include <quadrotor_input/RadioControlMixer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "radio_control_mixer");
  RadioControlMixer mix;

  ros::spin();
}



