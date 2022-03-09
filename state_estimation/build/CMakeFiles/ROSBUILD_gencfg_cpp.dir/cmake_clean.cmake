FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/state_estimation/msg"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/state_estimation/PoseGenConfig.h"
  "../docs/PoseGenConfig.dox"
  "../docs/PoseGenConfig-usage.dox"
  "../src/state_estimation/cfg/PoseGenConfig.py"
  "../docs/PoseGenConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
