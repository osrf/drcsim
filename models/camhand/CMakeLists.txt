set(MODEL_NAME camhand)
add_subdirectory(meshes)
add_subdirectory(ros)
set (files 
  model.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME})

