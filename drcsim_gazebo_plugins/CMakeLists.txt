cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo_plugins)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DRCVehiclePlugin DRCBuildingPlugin
)
find_package(catkin) # REQUIRED COMPONENTS nothing)

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
  ${PROJECT_SOURCE_DIR}/include  #hm, is this ok?
)

# compile and install gazebo plugins
add_library(DRCVehiclePlugin SHARED src/DRCVehiclePlugin.cc)
set_target_properties(DRCVehiclePlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
set_target_properties(DRCVehiclePlugin PROPERTIES LINK_FLAGS "${ld_flags}")
target_link_libraries(DRCVehiclePlugin ${GAZEBO_LIBRARIES})
install (TARGETS DRCVehiclePlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/plugins/)
#install (TARGETS DRCVehiclePlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${drcsim_gazebo_plugins_VERSION}/plugins/)

add_library(DRCBuildingPlugin SHARED src/DRCBuildingPlugin.cc)
set_target_properties(DRCBuildingPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
set_target_properties(DRCBuildingPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
target_link_libraries(DRCBuildingPlugin ${GAZEBO_LIBRARIES})
install (TARGETS DRCBuildingPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/plugins/)
#install (TARGETS DRCBuildingPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${drcsim_gazebo_plugins_VERSION}/plugins/)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
  #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
