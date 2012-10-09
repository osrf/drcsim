set(MODEL_NAME standpipe)
set (files 
  model.sdf
  manifest.xml
  standpipe.urdf
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
