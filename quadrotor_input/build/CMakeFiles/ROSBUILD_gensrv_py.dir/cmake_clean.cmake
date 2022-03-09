FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/quadrotor_input/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/quadrotor_input/srv/__init__.py"
  "../src/quadrotor_input/srv/_SelectStateSource.py"
  "../src/quadrotor_input/srv/_NotifyController.py"
  "../src/quadrotor_input/srv/_CommandController.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
