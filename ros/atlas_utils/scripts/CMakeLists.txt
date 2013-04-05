set (files
  gdbrun
  run_gazebo
  run_gazebo_paused
  run_gzserver
  run_gzserver_gdb
  reset_pose.py
  nanokontrol.py
  arm_teleop.py
  5steps.py
  test.py
  performance_test1.py
)
install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/scripts/)

