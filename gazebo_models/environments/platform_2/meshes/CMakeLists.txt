set (files 
  platform_2.dae
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/${MODEL_NAME}/meshes)
