set (files
  run_gazebo
  run_gzserver
  reset_pose.py
)
install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/scripts/)

