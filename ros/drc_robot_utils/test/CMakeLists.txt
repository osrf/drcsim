set (rostests
  drc_robot_no_controllers_publishers_hz
  drc_robot_publishers_hz
  drc_sim_v0_publishers_hz
) 
foreach (rostest ${rostests})
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
endforeach()
