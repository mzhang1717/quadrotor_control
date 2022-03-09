FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quadrotor_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/quadrotor_msgs/HeightWithVarianceStamped.h"
  "../msg_gen/cpp/include/quadrotor_msgs/State.h"
  "../msg_gen/cpp/include/quadrotor_msgs/DraganflyRadioControl.h"
  "../msg_gen/cpp/include/quadrotor_msgs/ControllerError.h"
  "../msg_gen/cpp/include/quadrotor_msgs/BatteryStatus.h"
  "../msg_gen/cpp/include/quadrotor_msgs/RadioControl.h"
  "../msg_gen/cpp/include/quadrotor_msgs/PressureAltitude.h"
  "../msg_gen/cpp/include/quadrotor_msgs/PositionDebug.h"
  "../msg_gen/cpp/include/quadrotor_msgs/HeightStamped.h"
  "../msg_gen/cpp/include/quadrotor_msgs/DirectionWithCovarianceStamped.h"
  "../msg_gen/cpp/include/quadrotor_msgs/StateDebug.h"
  "../msg_gen/cpp/include/quadrotor_msgs/PositionStamped.h"
  "../msg_gen/cpp/include/quadrotor_msgs/ControlInputs.h"
  "../msg_gen/cpp/include/quadrotor_msgs/AltitudeDebug.h"
  "../msg_gen/cpp/include/quadrotor_msgs/ControlWarn.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
