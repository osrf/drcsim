set(MODEL_NAME block_angle_base)
set (files 
  model.config
  model.sdf
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gazebo_models/environments/${MODEL_NAME}/)
