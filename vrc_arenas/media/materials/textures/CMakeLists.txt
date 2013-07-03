set (files 
  metal_plate.jpg
  target.png
  warning_wall.png
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/media/materials/textures/)
