set (files 
  atlas_commander.py
  gdbrun
  check_inertia_symmetry.bash
  run_gazebo
  run_gazebo_paused
  run_gzserver
  run_gzserver_gdb
  run_gzserver_perf
  run_gzserver_callgrind
  run_gzserver_memcheck
  run_gzserver_cheats
  reset_pose.py
  nanokontrol.py
  arm_teleop.py
  5steps.py
  stand.py
  test.py
  performance_test1.py
  keyboard_teleop.py
  orientation_checker.py
)
install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/scripts/)

