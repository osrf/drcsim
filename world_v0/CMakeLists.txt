set (worlds
     drc_sim_v0.world
    )
install(FILES ${worlds} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/worlds/)
