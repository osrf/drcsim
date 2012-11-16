set(MODEL_NAME drc_robot)
add_subdirectory(meshes)
add_subdirectory(ros)
set (files 
  model.sdf
  model_sandia_hand.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
