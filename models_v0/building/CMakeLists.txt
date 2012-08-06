cmake_minimum_required(VERSION 2.4.6)

#set(USE_ROS "1")

if(USE_ROS)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)

  rosbuild_init()

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  #uncomment if you have defined messages
  #rosbuild_genmsg()
  #uncomment if you have defined services
  #rosbuild_gensrv()

  #common commands for building c++ executables and libraries
  #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #rosbuild_add_boost_directories()
  #rosbuild_link_boost(${PROJECT_NAME} thread)
  #rosbuild_add_executable(example examples/example.cpp)
  #target_link_libraries(example ${PROJECT_NAME})

  #rosbuild_add_library(move_ragdoll plugins/move_ragdoll.cpp)

  # build model from xacro
  add_custom_command(OUTPUT models/building.model COMMAND rosrun ARGS xacro xacro.py models/building.model.xacro > models/building.model)
  set(generate_model_files models/building.model)
  add_custom_target(model_files ALL DEPENDS ${generate_model_files})

else(USE_ROS)


  # non-ros drc stuff
  set (models
       models/building.model
      )
  install(FILES ${models} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drc_sim-${DRC_SIM_VERSION}/models/)

  cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
  include (FindPkgConfig)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
    pkg_check_modules(GAZEBO_TRANSPORT gazebo_transport)
  endif()
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(modify_building SHARED plugins/modify_building.cpp)
  target_link_libraries(modify_building ${GAZEBO_libraries})

  # install plugins
  INSTALL(TARGETS modify_building DESTINATION lib/drc-${DRC_SIM_VERSION}/plugins)

endif(USE_ROS)
