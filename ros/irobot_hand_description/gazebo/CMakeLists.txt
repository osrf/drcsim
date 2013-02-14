add_subdirectory(irobot_hand)
set (files
  database.config
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/)

