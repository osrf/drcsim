set (files
  camhand.launch
  camhand_empty_world.launch
  upload.launch
  visualize.launch)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)
