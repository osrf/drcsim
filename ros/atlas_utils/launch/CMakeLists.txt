set (files
  upload_atlas.launch
  atlas_bringup.launch
  atlas.launch
  atlas_sandia_hands.launch
  atlas_irobot_hands.launch
  atlas_drc_vehicle_fire_hose.launch
  atlas_golf_cart_fire_hose.launch
  atlas_arm_controllers.launch
  atlas_position_controllers.launch
  atlas_no_controllers.launch
  atlas_sandia_hands_no_controllers.launch
  atlas_irobot_hands_no_controllers.launch
  drc_sim_v0.launch
  drc_trajectory_controllers.launch
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)

