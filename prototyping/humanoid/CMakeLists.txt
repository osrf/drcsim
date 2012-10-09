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
  #rosbuild_add_library(${PROJECT_NAME} src/example.cc)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #rosbuild_add_boost_directories()
  #rosbuild_link_boost(${PROJECT_NAME} thread)
  #rosbuild_add_executable(example examples/example.cc)
  #target_link_libraries(example ${PROJECT_NAME})

  rosbuild_add_library(move_model_test src/move_model_test.cc)
  rosbuild_add_library(pr2_pose_test src/pr2_pose_test.cc)

else(USE_ROS)

  # non-ros drc stuff
  # install model file
  set (models
       models/ragdoll.model
      )
  install(FILES ${models} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/)


  # install plugin for ragdoll
  add_library(move_ragdoll SHARED plugins/move_ragdoll.cc)
  target_link_libraries(move_ragdoll ${GAZEBO_libraries})
  INSTALL(TARGETS move_ragdoll DESTINATION lib/drc-${DRC_SIM_VERSION}/plugins)

  # compile a pose publisher for testing
  # Find Boost et. al., if not specified manually
  include(FindBoost)
  set (MIN_BOOST_VERSION 1.40.0 CACHE INTERNAL "Boost min version requirement" FORCE)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED thread signals system filesystem program_options regex)
  if (NOT Boost_FOUND)
    BUILD_ERROR ("Boost not found. Please install thread signals system filesystem program_options regex boost version ${MIN_BOOST_VERSION} or higher.")
  endif() 
  execute_process(COMMAND pkg-config --modversion protobuf 
    OUTPUT_VARIABLE PROTOBUF_VERSION
    RESULT_VARIABLE protobuf_modversion_failed)
  ########################################
  if (PROTOBUF_VERSION LESS 2.3.0)
    BUILD_ERROR("Incorrect version: Gazebo requires protobuf version 2.3.0 or greater")
  endif()
  ########################################
  # The Google Protobuf library for message generation + serialization
  find_package(Protobuf REQUIRED)
  if (NOT PROTOBUF_FOUND)
    BUILD_ERROR ("Missing: Google Protobuf (libprotobuf-dev)")
  endif()
  if (NOT PROTOBUF_PROTOC_EXECUTABLE)
    BUILD_ERROR ("Missing: Google Protobuf Compiler (protobuf-compiler)")
  endif()
  if (NOT PROTOBUF_PROTOC_LIBRARY)
    BUILD_ERROR ("Missing: Google Protobuf Compiler Library (libprotoc-dev)")
  endif()
  add_executable(pub_poses plugins/pub_poses.cc)
  target_link_libraries(pub_poses ${PROTOBUF_LIBRARY} ${GAZEBO_TRANSPORT_libraries} ${GAZEBO_libraries} ${Boost_LIBRARIES} pthread gazebo gazebo_transport gazebo_common gazebo_msgs)
  INSTALL(TARGETS pub_poses DESTINATION bin)


endif(USE_ROS)
