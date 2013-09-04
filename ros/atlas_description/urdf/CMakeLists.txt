set (files
  atlas.gazebo
  atlas.transmission
  atlas.urdf
  atlas_v3.gazebo
  atlas_v3.transmission
  atlas_v3.urdf
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/urdf/)

