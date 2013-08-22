cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo_plugins)

# catkin_package()
# find_package(catkin REQUIRED COMPONENTS nothing)

# Depend on system install of Gazebo and Boost
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

set (plugins_sources
  DRCVehiclePlugin
  DRCBuildingPlugin
)

# compile and install gazebo plugins
foreach (src ${plugins_sources})
  add_library(${src} SHARED src/${src}.cc)
  set_target_properties(${src} PROPERTIES COMPILE_FLAGS "${cxx_flags}")
  set_target_properties(${src} PROPERTIES LINK_FLAGS "${ld_flags}")
  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
  # install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
  install(TARGETS ${src} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
endforeach (src ${plugins_sources})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# install AtlasSimInterface Binary
install (FILES AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/lib64/libAtlasSimInterface.so.${ATLAS_SIM_INTERFACE_VERSION_FULL}
         DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
         RENAME libAtlasSimInterface.so)
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/include
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/doc
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})
