cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(ekf_node src/ekf_node.cpp src/ExtendedKalmanFilter.cpp src/FilterBase.cpp src/AircraftModel.cpp)
rosbuild_add_executable(ekf_calib_node src/ekf_node.cpp src/ExtendedKalmanFilter.cpp src/FilterBase.cpp src/AircraftModel.cpp)
rosbuild_add_executable(ekf_fixed_rel_node src/ekf_node.cpp src/ExtendedKalmanFilter.cpp src/FilterBase.cpp src/AircraftModel.cpp)
rosbuild_add_executable(simple_velocity_filter src/simple_velocity_filter.cpp src/SimpleVelocityFilter.cpp)
rosbuild_add_executable(pose_generator src/pose_generator.cpp)
rosbuild_add_executable(noisy_pose_guess_generator src/noisy_pose_guess_generator.cpp src/Sample.cpp)
rosbuild_add_executable(grab_calib_pose src/grab_calib_pose.cpp)
rosbuild_add_executable(write_calib_pose src/write_calib_pose.cpp)
rosbuild_add_executable(calc_pose_noise src/calc_pose_noise.cpp)
rosbuild_add_executable(calc_imu_noise src/calc_imu_noise.cpp)

rosbuild_add_compile_flags(ekf_node -Wall -std=c++0x -march=core2 -msse3 -O3)
rosbuild_add_compile_flags(ekf_calib_node -Wall -std=c++0x -march=core2 -msse3 -O3 -DCALIBRATION)
rosbuild_add_compile_flags(ekf_fixed_rel_node -Wall -std=c++0x -march=core2 -msse3 -O3 -DFIXED_REL_WORLD)

target_link_libraries(ekf_node yaml-cpp)
target_link_libraries(ekf_calib_node yaml-cpp)
target_link_libraries(ekf_fixed_rel_node yaml-cpp)
target_link_libraries(grab_calib_pose yaml-cpp)
target_link_libraries(pose_generator yaml-cpp)
target_link_libraries(noisy_pose_guess_generator yaml-cpp)
