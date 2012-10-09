set(MODEL_NAME low_poly_golf_cart_test)
set (files 
  model.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
