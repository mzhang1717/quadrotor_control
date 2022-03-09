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
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(joystick_converter src/joystick_converter.cpp src/JoystickConverter.cpp)
#rosbuild_add_executable(simple_state_machine src/simple_state_machine.cpp src/SimpleStateMachine.cpp)
rosbuild_add_executable(radio_control_mixer src/radio_control_mixer.cpp src/RadioControlMixer.cpp)
rosbuild_add_executable(draganfly_converter src/draganfly_converter.cpp src/DraganflyConverter.cpp)
#rosbuild_add_executable(altitude_controller src/altitude_controller.cpp src/ControllerBase.cpp src/AltitudeController.cpp)
rosbuild_add_executable(position_controller src/position_controller.cpp src/ControllerBase.cpp src/PositionController.cpp)
rosbuild_add_executable(multi_box src/multi_box.cpp src/MultiBox.cpp)
rosbuild_add_executable(virtual_px4 src/virtual_px4.cpp)
#target_link_libraries(example ${PROJECT_NAME})
