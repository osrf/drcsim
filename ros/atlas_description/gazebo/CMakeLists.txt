add_subdirectory(atlas)
set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)

