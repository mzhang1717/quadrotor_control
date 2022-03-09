cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(cfg/Controller.cfg)

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_library(gazebo_draganfly_x8 src/GazeboDraganflyX8.cpp src/QuadrotorPIDController.cpp)
rosbuild_link_boost(gazebo_draganfly_x8 thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(gazebo_ros_quadrotor_pid fluid_test)
