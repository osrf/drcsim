cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(NOT DEFINED ENV{ATLAS_ROBOT_INTERFACE_ROOT})
  message(FATAL_ERROR "You must set ATLAS_ROBOT_INTERFACE_ROOT to the directory
where you unpacked the AtlasRobotInterface distribution.")
endif()

include_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/include)
include_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/examples/atlas_api_control)
link_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/lib64)
rosbuild_add_boost_directories()

# We're compiling in a couple of files from the BDI example.  Not the cleanest
# thing to do, but better than copying ~1600 lines of codes.
rosbuild_add_executable(atlas_interface_bridge
  src/atlas_interface_bridge.cpp
  $ENV{ATLAS_ROBOT_INTERFACE_ROOT}/examples/atlas_api_control/AtlasUtility.cpp
  $ENV{ATLAS_ROBOT_INTERFACE_ROOT}/examples/atlas_api_control/ros_input_string.cpp)
target_link_libraries(atlas_interface_bridge AtlasRobotInterface)
# AtlasRobotInterface apparently depends on zlib
target_link_libraries(atlas_interface_bridge z)
rosbuild_link_boost(atlas_interface_bridge thread)
