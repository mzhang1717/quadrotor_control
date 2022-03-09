FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quadrotor_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/quadrotor_msgs/msg/__init__.py"
  "../src/quadrotor_msgs/msg/_HeightWithVarianceStamped.py"
  "../src/quadrotor_msgs/msg/_State.py"
  "../src/quadrotor_msgs/msg/_DraganflyRadioControl.py"
  "../src/quadrotor_msgs/msg/_ControllerError.py"
  "../src/quadrotor_msgs/msg/_BatteryStatus.py"
  "../src/quadrotor_msgs/msg/_RadioControl.py"
  "../src/quadrotor_msgs/msg/_PressureAltitude.py"
  "../src/quadrotor_msgs/msg/_PositionDebug.py"
  "../src/quadrotor_msgs/msg/_HeightStamped.py"
  "../src/quadrotor_msgs/msg/_DirectionWithCovarianceStamped.py"
  "../src/quadrotor_msgs/msg/_StateDebug.py"
  "../src/quadrotor_msgs/msg/_PositionStamped.py"
  "../src/quadrotor_msgs/msg/_ControlInputs.py"
  "../src/quadrotor_msgs/msg/_AltitudeDebug.py"
  "../src/quadrotor_msgs/msg/_ControlWarn.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
