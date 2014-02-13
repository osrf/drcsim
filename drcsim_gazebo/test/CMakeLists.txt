set (rostests
  atlas_publishers_hz.test
  atlas_sandia_hands_publishers_hz.test
  atlas_publishers_hz_gpu.test
  atlas_sandia_hands_publishers_hz_gpu.test
  atlas_rosapi.test
  atlas_sandia_hands_rosapi.test
  vrc_task_1_rosapi.test
  vrc_task_2_rosapi.test
  vrc_task_3_rosapi.test
  performance_test1.test
  atlas_cheats_rosapi.test
  atlas_sandia_hands_cheats_rosapi.test
  vrc_task_1_cheats_rosapi.test
  vrc_task_2_cheats_rosapi.test
  vrc_task_3_cheats_rosapi.test
  vrc_task_1_scoring.test
  vrc_task_2_scoring.test
  vrc_task_2_tricking_scoring.test
  vrc_task_1_start_standup.test
  vrc_task_2_start_standup.test
  vrc_task_3_start_standup.test
  vrc_task_1_gzlog_stop.test
  vrc_task_2_gzlog_stop.test
  vrc_task_3_gzlog_stop.test
  vrc_task_1_gzlog_stop_longrun.test
  vrc_task_1_gzlog_stop_longrun_zlib.test
  vrc_task_1_dynamic_walking.test
  vrc_task_2_dynamic_walking.test
  vrc_task_3_dynamic_walking.test
  multicamera_connection.test
)

# Only enable tests if we have a working GPU, which we use as a proxy for
# running on a machine that's powerful enough to hit our performance targets.
# Otherwise, we get periodic failiures from timing issues.
include (CheckDisplayCapabilities.cmake)
if(VALID_DRI_DISPLAY)
  foreach (rostest ${rostests})
    # We don't set a timeout here because we trust rostest to enforce the
    # timeout specified in each .test file.
    add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
    # Check for test result file and create one if needed.  rostest can fail to
    # generate a file if it throws an exception.
    add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  endforeach()
endif()

# Install test files because we'll run tests from the install tree.  We could
# get away with running most of the test from the source tree, but it's clearer
# and safer to run them from the install tree.  Plus we get to use things like
# $(find drcsim_gazebo) in rostest files.
install(FILES
  ${rostests}
  atlas_rosapi.yaml
  multisense_sl_rosapi.yaml
  sandia_hands_rosapi.yaml
  atlas_cheats_rosapi.yaml
  drc_vehicle_cheats_rosapi.yaml
  vrc_rosapi.yaml
  golf_cart_cheats_rosapi.yaml
  atlas_arenas_start_testing.launch
  atlas_arenas_logging.launch
  ros_subscribers.launch
  pub_atlas_command.launch
  perf_test.launch
  perf_test_local.launch
  vrc_task_1_commander.launch
  vrc_task_1_zlib_compression.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

install(PROGRAMS
  ros_api_checker
  run_gzstats.sh
  run_pub_atlas_command.sh
  run_recordmydesktop.sh
  run_rosbag.sh
  parse_logs.sh
  meanvar.py
  vrc_task_1_scoring_test
  vrc_task_2_scoring_test
  vrc_task_2_tricking_scoring_test
  gzlog_stop_checker.py
  vrc_walking_test
  multicamera_subscriber
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
