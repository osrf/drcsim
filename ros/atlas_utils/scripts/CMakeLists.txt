set (files
  gdbrun
  run_gazebo
  run_gazebo_paused
  run_gzserver
  run_gzserver_gdb
  run_gzstats_log
  reset_pose.py
  nanokontrol.py
  arm_teleop.py
  5steps.py
  stand.py
  stats_from_gzstats_log.py
  test.py
  performance_test1.py
)
install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/scripts/)

