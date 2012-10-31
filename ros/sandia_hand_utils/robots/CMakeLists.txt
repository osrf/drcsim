set (files
  sandia_hand.urdf.xacro)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/robots/)

