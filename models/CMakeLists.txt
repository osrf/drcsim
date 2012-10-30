add_subdirectory(sandiahand_left)
add_subdirectory(sandiahand_right)
add_subdirectory(drc_robot)
add_subdirectory(drc_terrain)
add_subdirectory(fire_hose)
add_subdirectory(low_poly_golf_cart)
add_subdirectory(standpipe)
add_subdirectory(irobothand_left)
add_subdirectory(irobothand_right)
set (files 
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/)
