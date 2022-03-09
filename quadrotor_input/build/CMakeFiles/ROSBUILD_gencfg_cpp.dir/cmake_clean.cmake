FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/quadrotor_input/srv"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/quadrotor_input/AltitudeControllerConfig.h"
  "../docs/AltitudeControllerConfig.dox"
  "../docs/AltitudeControllerConfig-usage.dox"
  "../src/quadrotor_input/cfg/AltitudeControllerConfig.py"
  "../docs/AltitudeControllerConfig.wikidoc"
  "../cfg/cpp/quadrotor_input/PositionControllerConfig.h"
  "../docs/PositionControllerConfig.dox"
  "../docs/PositionControllerConfig-usage.dox"
  "../src/quadrotor_input/cfg/PositionControllerConfig.py"
  "../docs/PositionControllerConfig.wikidoc"
  "../cfg/cpp/quadrotor_input/JoystickConverterConfig.h"
  "../docs/JoystickConverterConfig.dox"
  "../docs/JoystickConverterConfig-usage.dox"
  "../src/quadrotor_input/cfg/JoystickConverterConfig.py"
  "../docs/JoystickConverterConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
