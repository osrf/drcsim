add_subdirectory(drc_terrain)
add_subdirectory(drc_vehicle)
add_subdirectory(fire_hose)
add_subdirectory(gate)
add_subdirectory(golf_cart)
add_subdirectory(platform_1)
add_subdirectory(platform_2)
add_subdirectory(powerplant)
add_subdirectory(standpipe)
add_subdirectory(stepping_block)

set (files 
  database.config
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/gazebo_models/environments/)
