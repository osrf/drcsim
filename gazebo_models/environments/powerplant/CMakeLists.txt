set(MODEL_NAME powerplant)
add_subdirectory(meshes)
add_subdirectory(materials)
set (files 
  model.sdf
  model.config
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/${MODEL_NAME}/)
# In gazebo-1.4, the model manifests were renamed from manifest.xml to model.config
# For backwards compatibility, a duplicate manifest.xml will be installed
install(FILES model.config
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/${MODEL_NAME}/
        RENAME manifest.xml
)
