cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(move_model_test SHARED move_model_test.cpp)
target_link_libraries(move_model_test ${GAZEBO_libraries})

add_library(pr2_pose_test SHARED pr2_pose_test.cpp)
target_link_libraries(pr2_pose_test ${GAZEBO_libraries})

INSTALL(TARGETS move_model_test pr2_pose_test DESTINATION lib/drc/plugins)
