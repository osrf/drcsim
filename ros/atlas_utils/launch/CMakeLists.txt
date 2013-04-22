set (files
  atlas_bringup.launch
  atlas.launch
  atlas_sandia_hands.launch
  atlas_sandia_hands_bringup.launch
  atlas_irobot_hands.launch
  atlas_drc_vehicle_fire_hose.launch
  atlas_golf_cart_fire_hose.launch
  atlas_arm_controllers.launch
  atlas_position_controllers.launch
  atlas_no_controllers.launch
  drc_sim_v0.launch
  drc_trajectory_controllers.launch
  qual_task_1.launch
  qual_task_2.launch
  qual_task_3.launch
  qual_task_4.launch
  vrc_task_1.launch
  vrc_task_2.launch
  vrc_task_3.launch
  atlas_arm_slider_teleop.launch
  keyboard_teleop.launch
  atlas_arenas_logging.launch
  atlas_arenas_start_testing.launch
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/launch/)

