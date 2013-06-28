set (files 
  coupling_hexagon.dae
  nozzle.stl
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/${MODEL_NAME}/meshes)
