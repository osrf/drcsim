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

add_subdirectory(ros)
