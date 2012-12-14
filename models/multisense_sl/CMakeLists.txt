set(MODEL_NAME multisense_sl)
add_subdirectory(meshes)
set (files 
  MultiSense-SL_simplified2.URDF
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
