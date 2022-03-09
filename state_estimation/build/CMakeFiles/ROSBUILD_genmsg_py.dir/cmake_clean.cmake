FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/state_estimation/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/state_estimation/msg/__init__.py"
  "../src/state_estimation/msg/_AircraftStateCalibMsg.py"
  "../src/state_estimation/msg/_AircraftStateMsg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
