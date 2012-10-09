set (files 
  f0.stl
  f1.stl
  f2.stl
  palm.stl
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
