set (rostests
  atlas_publishers_hz
  atlas_sandia_hands_publishers_hz
  atlas_sandia_hands_performance
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
         atlas_publishers_hz_gpu
         # Pending hand camera implementation
         #atlas_sandia_hands_publishers_hz_gpu
         )
  endif()
endif()

foreach (rostest ${rostests})
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
endforeach()
