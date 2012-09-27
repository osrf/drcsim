set (plugins
  RubblePlugin
  DRCRobotPlugin
  DRCVehiclePlugin
  DRCBuildingPlugin
  DRCFirehosePlugin
)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/plugins/)
endforeach (src ${plugins})
