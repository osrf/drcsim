set(MODEL_NAME vrc_grass_plane2)
set (files 
  model.config
  model.sdf
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/${MODEL_NAME}/)
