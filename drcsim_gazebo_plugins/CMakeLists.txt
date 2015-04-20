cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo_plugins)
# set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin) # REQUIRED COMPONENTS nothing)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DRCVehiclePlugin DRCBuildingPlugin
)

# Depend on system install of Gazebo and Boost
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  ${PROJECT_SOURCE_DIR}/include  #hm, is this ok?
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# compile and install gazebo plugins
add_library(DRCVehiclePlugin SHARED src/DRCVehiclePlugin.cc)
target_link_libraries(DRCVehiclePlugin ${GAZEBO_LIBRARIES})
install (TARGETS DRCVehiclePlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(DRCBuildingPlugin SHARED src/DRCBuildingPlugin.cc)
target_link_libraries(DRCBuildingPlugin ${GAZEBO_LIBRARIES})
install (TARGETS DRCBuildingPlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)
