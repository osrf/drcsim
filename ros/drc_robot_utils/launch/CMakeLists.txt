set (files
  upload_drc_robot.launch
  drc_robot_bringup.launch
  drc_robot.launch
  drc_robot_golf_cart_fire_hose.launch
  drc_robot_arm_controllers.launch
  drc_robot_position_controllers.launch
  drc_robot_no_controllers.launch
  drc_robot_sandia_hand_no_controllers.launch
  drc_robot_irobot_hand_no_controllers.launch
  drc_sim_v0.launch
  drc_trajectory_controllers.launch
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)

