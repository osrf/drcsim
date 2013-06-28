option(LIGHT_TEST_SUITE "Light version of testsuite" ON)
option(HEAVY_TEST_SUITE "Heavy version of testsuite" OFF)
option(PARONOID_TEST_SUITE "Paranoid version of testsuite: all tests in all worls" OFF)

if (PARANOID_TEST_SUITE)
    message(STATUS "Paranoid test suite enabled")
elseif (HEAVY_TEST_SUITE) 
    message(STATUS "Heavy test suite enabled")
else()
    message(STATUS "Light test suite enabled")
endif()

# rostests will host one test for each type of world
set (rostests
  vrc_final_task1_rosapi.test
  vrc_final_task6_rosapi.test
  vrc_final_task11_rosapi.test
  vrc_final_task1_gzlog_stop.test
  vrc_final_task6_gzlog_stop.test
  vrc_final_task11_gzlog_stop.test
  vrc_final_task1_start_standup.test
  vrc_final_task6_start_standup.test
  vrc_final_task11_start_standup.test
  vrc_final_task1_scoring.test
  # Need to figure out what's make teleport to fall the robot
  # vrc_final_task6_scoring.test
  vrc_final_task7_scoring.test
  vrc_final_task1_atlas_pubs.test
  vrc_final_task1_atlas_pubs_gpu.test
  vrc_final_task1_hand_pubs.test
  vrc_final_task1_hand_pubs_gpu.test
)

if (HEAVY_TEST_SUITE OR PARANOID_TEST_SUITE)
  set (rostests
    ${rostests}
    # VRC 1 worlds
    vrc_final_task2_rosapi.test
    vrc_final_task3_rosapi.test
    vrc_final_task4_rosapi.test
    vrc_final_task5_rosapi.test
    vrc_final_task2_gzlog_stop.test
    vrc_final_task3_gzlog_stop.test
    vrc_final_task4_gzlog_stop.test
    vrc_final_task5_gzlog_stop.test
    vrc_final_task1_gzlog_stop_30min_run.test
    vrc_final_task2_start_standup.test
    vrc_final_task3_start_standup.test
    vrc_final_task4_start_standup.test
    vrc_final_task5_start_standup.test
    vrc_final_task2_scoring.test
    vrc_final_task3_scoring.test
    # 4 and 5 need proper pose teleport
    # vrc_final_task4_scoring.test
    # vrc_final_task5_scoring.test

    # VRC 2 worlds
    vrc_final_task7_rosapi.test
    vrc_final_task8_rosapi.test
    vrc_final_task9_rosapi.test
    vrc_final_task10_rosapi.test
    vrc_final_task7_gzlog_stop.test
    vrc_final_task8_gzlog_stop.test
    vrc_final_task9_gzlog_stop.test
    vrc_final_task10_gzlog_stop.test
    vrc_final_task7_start_standup.test
    vrc_final_task8_start_standup.test
    vrc_final_task9_start_standup.test
    vrc_final_task10_start_standup.test
    # vrc_final_task7_scoring.test
    vrc_final_task8_scoring.test
    vrc_final_task9_scoring.test
    vrc_final_task10_scoring.test
    vrc_final_task2_atlas_pubs.test
    vrc_final_task2_atlas_pubs_gpu.test

    # VRC 3 worlds
    vrc_final_task12_rosapi.test
    vrc_final_task13_rosapi.test
    vrc_final_task14_rosapi.test
    vrc_final_task15_rosapi.test
    vrc_final_task12_gzlog_stop.test
    vrc_final_task13_gzlog_stop.test
    vrc_final_task14_gzlog_stop.test
    vrc_final_task15_gzlog_stop.test
    vrc_final_task12_start_standup.test
    vrc_final_task13_start_standup.test
    vrc_final_task14_start_standup.test
    vrc_final_task15_start_standup.test
    vrc_final_task3_hand_pubs.test
    vrc_final_task3_hand_pubs_gpu.test

  )
endif()

if (PARANOID_TEST_SUITE)
  set (rostests
    ${rostests}
    vrc_final_task2_gzlog_stop_30min_run.test
    vrc_final_task3_gzlog_stop_30min_run.test
    vrc_final_task4_gzlog_stop_30min_run.test
    vrc_final_task5_gzlog_stop_30min_run.test
    vrc_final_task6_gzlog_stop_30min_run.test
    vrc_final_task7_gzlog_stop_30min_run.test
    vrc_final_task8_gzlog_stop_30min_run.test
    vrc_final_task9_gzlog_stop_30min_run.test
    vrc_final_task10_gzlog_stop_30min_run.test
    vrc_final_task11_gzlog_stop_30min_run.test
    vrc_final_task12_gzlog_stop_30min_run.test
    vrc_final_task13_gzlog_stop_30min_run.test
    vrc_final_task14_gzlog_stop_30min_run.test
    vrc_final_task15_gzlog_stop_30min_run.test
    vrc_final_task2_hand_pubs.test
    vrc_final_task2_hand_pubs_gpu.test
    vrc_final_task3_atlas_pubs.test
    vrc_final_task3_atlas_pubs_gpu.test
  ) 
endif()
  

foreach (rostest ${rostests})
  # We don't set a timeout here because we trust rostest to enforce the
  # timeout specified in each .test file.
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
endforeach()

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ros/vrc_finals/test/)
