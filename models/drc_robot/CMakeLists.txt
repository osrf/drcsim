set(MODEL_NAME drc_robot)
add_subdirectory(meshes)
add_subdirectory(ros)
add_subdirectory(materials)
set (files 
  model.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
