set (files 
  drc_robot.model
  low_poly_golf_cart.model
  low_poly_golf_cart1.model
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/)
