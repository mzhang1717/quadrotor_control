#include <quadrotor_input/JoystickConverter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_converter");
  JoystickConverter joystick_converter;

  ros::spin();
}



