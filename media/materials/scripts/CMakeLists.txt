set (files 
  drc_sim.material
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/media/materials/scripts/)
