set (files 
finger_base.stl
finger_distal.stl
finger_proximal.stl
palm.stl
)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/irobot_hand/meshes/)
