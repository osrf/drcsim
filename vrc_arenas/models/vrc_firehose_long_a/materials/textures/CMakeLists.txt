set (files 
  canvas_a.jpg
  coupling_hexagon.png
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/${MODEL_NAME}/materials/textures)
