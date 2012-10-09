set (files 
  drc_sim_v0_terrain.png
  san_fauxcity.png
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/media/materials/textures/)
