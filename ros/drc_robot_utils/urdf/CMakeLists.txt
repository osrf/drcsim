set (files
  drc_robot.gazebo
  drc_robot.transmission
  drc_robot.urdf
  drc_robot.sensors
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/urdf/)

