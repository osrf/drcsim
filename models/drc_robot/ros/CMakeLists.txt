set (files 
position_controllers.yaml
arm_controllers.yaml
leg_torso_position_controllers.yaml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/ros/)
