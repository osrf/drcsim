set (worlds
     drc_sim_v0.world
     atlas.world
     atlas_irobot_hand.world
     atlas_sandia_hand.world
     atlas_golf_cart_fire_hose.world
     golf_cart.world
     fire_hose.world
     powerplant.world
    )
install(FILES ${worlds} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/worlds/)
