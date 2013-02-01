add_subdirectory(meshes)
set(MODEL_NAME irobot_hand)
set (files
  model.config
)

install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/${MODEL_NAME})
