set (rostests
  atlas_publishers_hz.test
  atlas_sandia_hands_publishers_hz.test
  performance_test1.launch
) 

# If we have a GPU, add in the GPU-dependent tests.
include (CheckDisplayCapabilities.cmake)

if(VALID_DRI_DISPLAY)
    list(APPEND rostests 
         atlas_arenas_logging.launch
         atlas_arenas_start_testing.launch
         atlas_publishers_hz_gpu.test
         atlas_rosapi.test
         atlas_sandia_hands_rosapi.test
         atlas_drc_vehicle_fire_hose_rosapi.test
         atlas_golf_cart_fire_hose_rosapi.test
         drc_sim_v0_rosapi.test
         gzlog_qual_1_log.test
         gzlog_qual_2_log.test
         gzlog_qual_3_log.test
         gzlog_qual_4_log.test
         gzlog_long_run.test
         qual_task_1_rosapi.test
         qual_task_2_rosapi.test
         qual_task_3_rosapi.test
         qual_task_4_rosapi.test
         vrc_task_1_rosapi.test
         vrc_task_2_rosapi.test
         vrc_task_3_rosapi.test
         # Pending hand camera implementation
         #atlas_sandia_hands_publishers_hz_gpu.test
         )
endif()

foreach (rostest ${rostests})
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
endforeach()

# Install test files because we'll run tests from the install tree.  We could
# get away with running most of the test from the source tree, but it's clearer
# and safer to run them from the install tree.  Plus we get to use things like
# $(find atlas_utils) in rostest files.
set(files
  ${rostests}
  atlas_rosapi.yaml
  multisense_sl_rosapi.yaml
  sandia_hands_rosapi.yaml
  atlas_cheats_rosapi.yaml
  drc_vehicle_cheats_rosapi.yaml
  golf_cart_cheats_rosapi.yaml
  ros_subscribers.launch
  pub_atlas_command.launch
  perf_test.launch
)
set(scripts
  ros_api_checker
  run_gzstats.sh
  run_pub_atlas_command.sh
  run_recordmydesktop.sh
  run_rosbag.sh
  parse_logs.sh
  meanvar.py
  gzlog_stop_checker.py
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/test/)
install(PROGRAMS ${scripts} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/test/)

