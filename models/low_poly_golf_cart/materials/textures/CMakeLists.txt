set (files 
  texture0.jpg
  texture1.jpg
  texture2.jpg
  texture3.jpg
  texture4.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/media/models/low_poly_golf_cart/images/)
