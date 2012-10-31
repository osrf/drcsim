add_subdirectory(sandia_hand_left)
add_subdirectory(sandia_hand_right)
add_subdirectory(sandia_hand)
add_subdirectory(drc_robot)
add_subdirectory(drc_terrain)
add_subdirectory(fire_hose)
add_subdirectory(low_poly_golf_cart)
add_subdirectory(standpipe)
add_subdirectory(irobot_hand)
set (files 
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/)
