cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

if(NOT DEFINED ENV{ATLAS_ROBOT_INTERFACE_ROOT})
  message(FATAL_ERROR "You must set ATLAS_ROBOT_INTERFACE_ROOT to the directory
where you unpacked the AtlasRobotInterface distribution.")
endif()

include_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/include)
link_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/lib64)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(atlas_interface src/atlas_interface.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(atlas_interface thread)
target_link_libraries(atlas_interface AtlasRobotInterface)
# AtlasRobotInterface apparently depends on zlib
target_link_libraries(atlas_interface z)

rosbuild_add_executable(example examples/example.cpp)
target_link_libraries(example atlas_interface)
