set (files 
  base_link.STL
  hokuyo_link.STL
)
install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/multisense_sl/meshes/)
