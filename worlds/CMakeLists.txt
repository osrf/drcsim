set (worlds
     drc_sim_v0.world
     drc_robot.world
     drc_robot_irobot_hand.world
     drc_robot_sandia_hand.world
     drc_robot_golf_cart.world
     golf_cart.world
     fire_hose.world
     powerplant.world
    )
install(FILES ${worlds} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/worlds/)
