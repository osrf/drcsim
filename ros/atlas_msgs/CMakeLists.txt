cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ENV{ROS_PACKAGE_PATH} ${DRC_SIM_CMAKE_SOURCE_DIR}/ros/atlas_msgs:${ROS_PACKAGE_PATH})
set(ENV{ROS_PACKAGE_PATH} ${DRC_SIM_CMAKE_SOURCE_DIR}/ros:${ROS_PACKAGE_PATH})
set(ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
message(status "\n\n\n$ENV{ROS_PACKAGE_PATH}\n\n\n")

set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
message(status "\n\n\n PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}\n\n\n")

project(atlas_msgs)
rosbuild_init()
# Note: recovering PROJECT_NAME here to drcsim leads to build failure

#find gazebo include (FindPkgConfig)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

message(STATUS "\n\n\nGID:  ${GAZEBO_INCLUDE_DIRS}\n\n")

# depends on DRCVehiclePlugin
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${DRC_SIM_CMAKE_SOURCE_DIR}/plugins
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PLUGIN_LINK_DIRECTORIES}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
INSTALL(CODE "EXECUTE_PROCESS(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)")

rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_boost_directories()

# needs pr2_controller_manager
rosbuild_add_library(gazebo_ros_controller_manager gazebo_ros_controller_manager.cpp)
rosbuild_link_boost(gazebo_ros_controller_manager thread)

# needs roscpp, trajectory_msgs
rosbuild_add_executable(pub_joint_trajectory_test pub_joint_trajectory_test.cpp)
rosbuild_add_executable(pub_joint_commands pub_joint_commands.cpp)

# needs std_msgs, sensor_msgs, geometry_msgs, image_transport tf, roscpp, trajectory_msgs
rosbuild_add_library(gazebo_ros_camera_utils gazebo_ros_camera_utils.cpp)
target_link_libraries(gazebo_ros_camera_utils CameraPlugin)
rosbuild_add_library(gazebo_ros_camera gazebo_ros_camera.cpp)
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils CameraPlugin)

rosbuild_add_library(MultiCameraPlugin MultiCameraPlugin.cc)
target_link_libraries(MultiCameraPlugin gazebo_sensors gazebo_rendering)
rosbuild_add_library(gazebo_ros_multicamera gazebo_ros_multicamera.cpp)
target_link_libraries(gazebo_ros_multicamera gazebo_ros_camera_utils MultiCameraPlugin)

rosbuild_add_library(gazebo_ros_laser gazebo_ros_laser.cpp)
target_link_libraries(gazebo_ros_laser RayPlugin)

rosbuild_add_library(gazebo_ros_imu gazebo_ros_imu.cpp)

rosbuild_add_library(gazebo_ros_joint_trajectory gazebo_ros_joint_trajectory.cpp)

rosbuild_add_library(gazebo_ros_api_plugin gazebo_ros_api_plugin.cpp)

rosbuild_add_library(gazebo_ros_p3d gazebo_ros_p3d.cpp)

rosbuild_add_library(MultiSenseSLPlugin MultiSenseSLPlugin.cpp)
rosbuild_add_library(SandiaHandPlugin SandiaHandPlugin.cpp)
rosbuild_add_library(AtlasPlugin AtlasPlugin.cpp)

# needs actionlib, control_msgs
rosbuild_add_library(VRCPlugin VRCPlugin.cpp)
target_link_libraries(VRCPlugin)

# needs DRCVehiclePlugin from non-ros plugins
rosbuild_add_library(DRCVehicleROSPlugin DRCVehicleROSPlugin.cpp)
target_link_libraries(DRCVehicleROSPlugin DRCVehiclePlugin)

install(FILES manifest.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
install(DIRECTORY msg msg_gen src DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME} PATTERN ".*" EXCLUDE)
