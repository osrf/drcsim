set (ROS_PACKAGE_NAME multisense_sl_description)
set (GAZEBO_INSTALL_PATH share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/${ROS_PACKAGE_NAME})

#set (ROS_PKG_INSTALL_PATH share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME})
#set (files
#  manifest.xml
#)
#install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/)

add_subdirectory(robots)
add_subdirectory(urdf)
add_subdirectory(gazebo)
add_subdirectory(meshes)
