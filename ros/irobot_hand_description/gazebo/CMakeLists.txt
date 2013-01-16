add_subdirectory(irobot_hand)
set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/)

