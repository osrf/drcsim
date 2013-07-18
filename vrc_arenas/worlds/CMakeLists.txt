set (worlds 
vrc_final_task1.world
vrc_final_task2.world
vrc_final_task3.world
vrc_final_task4.world
vrc_final_task5.world
vrc_final_task6.world
vrc_final_task7.world
vrc_final_task8.world
vrc_final_task9.world
vrc_final_task10.world
vrc_final_task11.world
vrc_final_task12.world
vrc_final_task13.world
vrc_final_task14.world
vrc_final_task15.world
)

install(FILES ${worlds} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/worlds/)
