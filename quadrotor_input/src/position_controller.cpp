#include <quadrotor_input/PositionController.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_controller");
  PositionController ctrl;
  ctrl.Run();
}



