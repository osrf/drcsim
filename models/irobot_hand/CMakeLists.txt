set(MODEL_NAME irobot_hand)
add_subdirectory(meshes)
set (files 
  model.urdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
