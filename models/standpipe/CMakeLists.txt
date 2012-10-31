set(MODEL_NAME standpipe)
add_subdirectory(meshes)
set (files 
  model.sdf
  manifest.xml
  standpipe.urdf
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
