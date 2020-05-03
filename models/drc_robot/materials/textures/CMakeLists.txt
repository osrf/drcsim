set (files 
  atlas_DRC_1.png
  atlas_DRC_carbon_fiber.png
  atlas_DRC_dark_1.png
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/materials/textures/)
