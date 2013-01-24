set (files
  #irobot_hand.gazebo.xacro
  #irobot_hand.transmission.xacro
  irobot_hand.urdf.xacro
  #irobot_hand.sensors.xacro
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/urdf/)

