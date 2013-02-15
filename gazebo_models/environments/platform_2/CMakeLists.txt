set(MODEL_NAME platform_2)

set (files 
  manifest.xml
  model-1_3.sdf
)

add_subdirectory(meshes)
add_subdirectory(materials)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/${MODEL_NAME}/)
