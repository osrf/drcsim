set (files 
position_controllers.yaml
arm_controllers.yaml
leg_controllers.yaml
torso_position_controllers.yaml
leg_torso_position_controllers.yaml
whole_body_trajectory_controller.yaml
arm_controllers_v3.yaml
leg_controllers_v3.yaml
leg_torso_position_controllers_v3.yaml
position_controllers_v3.yaml
torso_position_controllers_v3.yaml
whole_body_trajectory_controller_v3.yaml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/config/)
