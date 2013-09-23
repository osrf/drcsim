cmake_minimum_required(VERSION 2.4.6)

set(ENV{ROS_ROOT} ${ROS_ROOT})
set(ENV{ROS_PACKAGE_PATH} ${ROS_PACKAGE_PATH})
# Push the PKG_CONFIG_PATH we were passed up into the environment, to allow us to find gazebo
set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(joint_commands_gui)
rosbuild_init()

set (CMAKE_INSTALL_RPATH_USE_LINK_PATH 1)

install(FILES manifest.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
install(FILES joint_command_gui.py DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
install(FILES joint_command_gui.png DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${DRC_SIM_PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${PROJECT_NAME})
