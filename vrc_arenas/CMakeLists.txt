cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr CACHE STRING "Install path prefix")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

add_subdirectory(media)
add_subdirectory(worlds)
add_subdirectory(models)
add_subdirectory(ros)

