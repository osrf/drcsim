set (files
  #irobot_hand.gazebo
  #irobot_hand.transmission
  irobot_hand.urdf
  #irobot_hand.sensors
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/urdf/)

