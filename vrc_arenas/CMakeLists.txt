cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr CACHE STRING "Install path prefix")

project (vrc_arenas)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

add_subdirectory(media)
add_subdirectory(worlds)
add_subdirectory(models)
add_subdirectory(ros)

#####################################
# Configure the setup script
configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/)
