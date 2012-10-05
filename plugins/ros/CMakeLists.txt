include(cmake/rospack.cmake)

get_rospack_flags(pr2_controller_manager)
include_directories(${pr2_controller_manager_INCLUDE_DIRS})
link_directories(${pr2_controller_manager_LIBRARY_DIRS})

# use, i.e. don't skip the full RPATH for the build tree
#SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
#SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${pr2_controller_manager_LIBRARY_DIRS}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
#SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
#LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
#IF("${isSystemDir}" STREQUAL "-1")
#   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#ENDIF("${isSystemDir}" STREQUAL "-1")

add_library(gazebo_ros_controller_manager SHARED gazebo_ros_controller_manager.cpp)
set_target_properties(gazebo_ros_controller_manager PROPERTIES COMPILE_FLAGS "${pr2_controller_manager_CFLAGS_OTHER}")
set_target_properties(gazebo_ros_controller_manager PROPERTIES LINK_FLAGS "${pr2_controller_manager_LDFLAGS_OTHER}")
target_link_libraries(gazebo_ros_controller_manager ${pr2_controller_manager_LIBRARIES})

install (TARGETS gazebo_ros_controller_manager DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/plugins/)

#set (ros_plugins
#  test_ros_plugin
#)
#
#foreach (src ${ros_plugins})
#  add_library(${src} SHARED ${src}.cc)
#  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
#  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/ros_plugins/)
#endforeach (src ${ros_plugins})
#
