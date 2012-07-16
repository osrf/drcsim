cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project (drc_sim)

set (DRC_SIM_MAJOR_VERSION 0)
set (DRC_SIM_MINOR_VERSION 1)
set (DRC_SIM_PATCH_VERSION 0)
set (DRC_SIM_VERSION ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION}.${DRC_SIM_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${DRC_SIM_VERSION}")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_subdirectory(plugins)
