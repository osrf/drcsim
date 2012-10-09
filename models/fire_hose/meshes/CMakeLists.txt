set (files 
  coupling.dae
  coupling2.dae
  nozzle.dae
  nozzle2.dae
  standpipe.dae
  standpipe2.dae
  coupling.stl
  coupling2.stl
  nozzle.stl
  nozzle2.stl
  standpipe.stl
  standpipe2.stl
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
