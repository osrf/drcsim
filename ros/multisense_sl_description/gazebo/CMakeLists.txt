add_subdirectory(multisense_sl)
set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)

