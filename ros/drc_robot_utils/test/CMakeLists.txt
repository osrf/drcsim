set (rostests
  publishers_hz
) 
foreach (rostest ${rostests})
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest}.test)
endforeach()
