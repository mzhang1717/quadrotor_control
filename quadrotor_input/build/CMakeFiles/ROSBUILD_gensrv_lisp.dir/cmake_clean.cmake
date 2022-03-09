FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/quadrotor_input/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SelectStateSource.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SelectStateSource.lisp"
  "../srv_gen/lisp/NotifyController.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_NotifyController.lisp"
  "../srv_gen/lisp/CommandController.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_CommandController.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
