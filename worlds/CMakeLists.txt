set (worlds
     drc_sim_v0.world
     drc_robot_golf_cart.world
    )
install(FILES ${worlds} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/worlds/)
