set(MODEL_NAME atlas)
add_subdirectory(meshes)
add_subdirectory(ros)
add_subdirectory(materials)
set (files 
  model.urdf
  model.urdf.square_foot
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
