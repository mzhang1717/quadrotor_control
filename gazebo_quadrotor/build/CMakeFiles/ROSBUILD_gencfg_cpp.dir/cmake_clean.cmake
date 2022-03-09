FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/gazebo_quadrotor/QuadrotorPIDControllerConfig.h"
  "../docs/QuadrotorPIDControllerConfig.dox"
  "../docs/QuadrotorPIDControllerConfig-usage.dox"
  "../src/gazebo_quadrotor/cfg/QuadrotorPIDControllerConfig.py"
  "../docs/QuadrotorPIDControllerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
