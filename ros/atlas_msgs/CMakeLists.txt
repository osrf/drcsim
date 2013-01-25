cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# save project name, paths
set(SAVE_PROJECT_NAME ${PROJECT_NAME})

set(ROSBUILD_DONT_REDEFINE_PROJECT 1)
project(atlas_msgs)
rosbuild_init()
# Note: recovering PROJECT_NAME here to drcsim leads to build failure

# depends on DRCVehiclePlugin
include_directories(
  ${CMAKE_SOURCE_DIR}/plugins
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
INSTALL(CODE "EXECUTE_PROCESS(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)")

rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_boost_directories()

# needs pr2_controller_manager
rosbuild_add_library(gazebo_ros_controller_manager gazebo_ros_controller_manager.cpp)
rosbuild_link_boost(gazebo_ros_controller_manager thread)

# needs roscpp, trajectory_msgs
rosbuild_add_executable(pub_joint_trajectory_test pub_joint_trajectory_test.cpp)
rosbuild_add_executable(pub_joint_commands pub_joint_commands.cpp)
target_link_libraries(pub_joint_commands gazebo_common rt)

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

install(FILES manifest.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
install(DIRECTORY msg msg_gen src DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME} PATTERN ".*" EXCLUDE)
