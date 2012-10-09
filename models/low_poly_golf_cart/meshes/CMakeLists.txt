set (files 
  model.dae
  no_wheels.dae
  no_wheels.stl
  no_wheels_test.dae
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME}/models/)
