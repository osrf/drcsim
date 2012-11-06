set (files 
  standpipe.dae
  standpipe2.dae
  standpipe.stl
  standpipe2.stl
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/meshes/)
