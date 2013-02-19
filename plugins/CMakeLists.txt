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

install (FILES AtlasSimInterface_1.0.3/lib64/libAtlasSimInterface.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
install (DIRECTORY AtlasSimInterface_1.0.3/config DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/config)
