add_subdirectory(no_wheels)
add_subdirectory(models)
add_subdirectory(images)
set (files 
  textures.txt
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/media/models/low_poly_golf_cart/)
