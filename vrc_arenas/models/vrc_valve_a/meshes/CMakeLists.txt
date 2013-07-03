set (files 
  vrc_valve_a.dae
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/${MODEL_NAME}/meshes)
