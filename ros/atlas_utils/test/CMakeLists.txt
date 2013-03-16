set (rostests
  atlas_publishers_hz.test
  atlas_sandia_hands_publishers_hz.test
  atlas_rosapi.test
  atlas_sandia_hands_rosapi.test
  performance_test1.launch
) 

# If we have a GPU, add in the GPU-dependent tests.
# TODO: replace this with the stuff that Jose's working on for inclusion in Gazebo
find_program(GLXINFO_PATH glxinfo)
if(GLXINFO_PATH)
  execute_process(COMMAND glxinfo
                  RESULT_VARIABLE glxinfo_returncode
                  OUTPUT_VARIABLE glxinfo_stdout
                  ERROR_VARIABLE glxinfo_stderr)
  if(NOT RESULT_VARIABLE)
    list(APPEND rostests 
         atlas_publishers_hz_gpu.test
         # Pending hand camera implementation
         #atlas_sandia_hands_publishers_hz_gpu.test
         )
  endif()
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
)
set(scripts
  ros_api_checker
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/test/)
install(PROGRAMS ${scripts} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/test/)

