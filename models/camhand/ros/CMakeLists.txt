set (files 
position_controllers.yaml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/ros/)
