set (files
  sandia_hand.launch
  sandia_hand_empty_world.launch
  upload.launch
  visualize.launch)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)
