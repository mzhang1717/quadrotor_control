FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/quadrotor_input/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/quadrotor_input/SelectStateSource.h"
  "../srv_gen/cpp/include/quadrotor_input/NotifyController.h"
  "../srv_gen/cpp/include/quadrotor_input/CommandController.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
