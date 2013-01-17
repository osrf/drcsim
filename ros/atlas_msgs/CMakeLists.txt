cmake_minimum_required(VERSION 2.4.6)
set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH})
set(ENV{ROS_PACKAGE_PATH} ${DRC_SIM_CMAKE_SOURCE_DIR}/ros/atlas_msgs:${ROS_PACKAGE_PATH})
set(ENV{ROS_PACKAGE_PATH} ${DRC_SIM_CMAKE_SOURCE_DIR}/ros:${ROS_PACKAGE_PATH})
set(ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
message(status "\n\n\n$ENV{ROS_PACKAGE_PATH}\n\n\n")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# save project name, paths
#set(SAVE_PROJECT_NAME ${PROJECT_NAME})

#set(ROSBUILD_DONT_REDEFINE_PROJECT 1)
#project(atlas_msgs)
rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/bin/)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#INSTALL(CODE "EXECUTE_PROCESS(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)")

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set (ROS_PKG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
