set (files
  vrc_final.launch
  vrc_final_task1.launch
  vrc_final_task2.launch
  vrc_final_task3.launch
  vrc_final_task4.launch
  vrc_final_task5.launch
  vrc_final_task6.launch
  vrc_final_task7.launch
  vrc_final_task8.launch
  vrc_final_task9.launch
  vrc_final_task10.launch
  vrc_final_task11.launch
  vrc_final_task12.launch
  vrc_final_task13.launch
  vrc_final_task14.launch
  vrc_final_task15.launch
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/vrc_finals/launch/)
