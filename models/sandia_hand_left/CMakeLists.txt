set(MODEL_NAME sandia_hand_left)
add_subdirectory(ros)
set (files 
  model.urdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)

