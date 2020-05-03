set (plugins
  #AtlasPlugin
  DRCVehiclePlugin
  DRCBuildingPlugin
  #DRCFirehosePlugin
)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
endforeach (src ${plugins})

install (FILES AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/lib64/libAtlasSimInterface.${ATLAS_SIM_INTERFACE_VERSION_FULL} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL} RENAME libAtlasSimInterface.so)
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}-${DRC_SIM_VERSION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/doc DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})
