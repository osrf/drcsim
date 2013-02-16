add_subdirectory(atlas)
set (files
  database.config
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)

