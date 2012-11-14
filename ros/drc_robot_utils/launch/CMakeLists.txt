set (files
  upload_drc_robot.launch
  drc_robot.launch
  drc_robot_arm_controllers.launch
  drc_robot_position_controllers.launch
  drc_robot_no_controllers.launch
  drc_sim_v0.launch
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)

