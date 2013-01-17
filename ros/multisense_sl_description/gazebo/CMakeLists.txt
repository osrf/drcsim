add_subdirectory(multisense_sl)
set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/)

