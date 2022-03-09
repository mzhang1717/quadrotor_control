FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/fsm/fsm_monitorConfig.h"
  "../docs/fsm_monitorConfig.dox"
  "../docs/fsm_monitorConfig-usage.dox"
  "../src/fsm/cfg/fsm_monitorConfig.py"
  "../docs/fsm_monitorConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
