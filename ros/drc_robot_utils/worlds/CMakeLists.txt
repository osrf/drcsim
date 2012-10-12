set (files
  drc_robot.world
  drc_robot_animate.world
  drc_robot_golf_cart.world
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/worlds/)

