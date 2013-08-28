set(MODEL_NAME standpipe)
add_subdirectory(meshes)
set (files 
  model.sdf
  model.config
  standpipe.urdf
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gazebo_models/environments/${MODEL_NAME}/)
