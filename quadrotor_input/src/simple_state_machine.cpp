#include <ros/ros.h>
#include <quadrotor_input/SimpleStateMachine.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_state_machine");
  SimpleStateMachine ssm;

  ros::spin();
}



