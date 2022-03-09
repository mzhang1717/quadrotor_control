#include <quadrotor_input/AltitudeController.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "altitude_controller");
  AltitudeController ctrl;
  ctrl.Run();
}



