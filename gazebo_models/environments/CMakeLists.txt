add_subdirectory(drc_terrain)
add_subdirectory(fire_hose)
add_subdirectory(golf_cart)
add_subdirectory(standpipe)
add_subdirectory(powerplant)
set (files 
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/)
