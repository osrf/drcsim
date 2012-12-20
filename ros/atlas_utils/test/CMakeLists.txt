set (rostests
  atlas_no_controllers_publishers_hz
  atlas_publishers_hz
  drc_sim_v0_publishers_hz
) 
foreach (rostest ${rostests})
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
endforeach()
