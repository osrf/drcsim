set(MODEL_NAME drc_robot)
add_subdirectory(meshes)
set (files 
  model.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/${MODEL_NAME})
