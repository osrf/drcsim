cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(GAZEBO_TRANSPORT gazebo_transport)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(move_ragdoll SHARED move_ragdoll.cpp)
target_link_libraries(move_ragdoll ${GAZEBO_libraries})

# install plugins
INSTALL(TARGETS move_ragdoll DESTINATION lib/drc/plugins)

# compile a pose publisher
# Find Boost, if not specified manually
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


add_executable(pub_poses pub_poses.cpp)
target_link_libraries(pub_poses ${PROTOBUF_LIBRARY} ${GAZEBO_TRANSPORT_libraries} ${GAZEBO_libraries} ${Boost_LIBRARIES} pthread gazebo gazebo_transport gazebo_common gazebo_msgs)
INSTALL(TARGETS pub_poses DESTINATION bin)

