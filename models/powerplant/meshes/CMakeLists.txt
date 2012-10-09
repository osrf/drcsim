set (files 
  powerplant.dae
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME}/meshes/)
