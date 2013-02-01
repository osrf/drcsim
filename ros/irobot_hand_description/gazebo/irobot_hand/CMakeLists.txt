add_subdirectory(meshes)
set (files
  model.config
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)
