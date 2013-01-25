add_subdirectory(irobot_hand)
set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)

