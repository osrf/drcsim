cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project (drc_sim)

set (DRC_SIM_MAJOR_VERSION 0)
set (DRC_SIM_MINOR_VERSION 1)
set (DRC_SIM_PATCH_VERSION 0)
set (DRC_SIM_VERSION ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION}.${DRC_SIM_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${DRC_SIM_VERSION}")


set (INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}-${DRC_SIM_VERSION}/${PROJECT_NAME}") 
set (LIB_INSTALL_DIR "lib" CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)") 
set (BIN_INSTALL_DIR "bin" CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)") 

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "RELEASE" CACHE STRING 
    "Choose the type of build, options are: debug release profile check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#####################################
# Configure the setup script

# We need the path to Gazebo's setup.sh file so that we can source it within our own setup.sh.
# Not sure how to get it other than inferring from other vars.
if(NOT GAZEBO_INCLUDE_DIRS)
  # Gazebo didn't give us any include dirs, which means that it's in /usr
  set(GAZEBO_INSTALL_PREFIX /usr)
else()
  # Look above the first include dir
  list(GET GAZEBO_LIBRARY_DIRS 0 GAZEBO_LIBRARY_DIR0)
  set(GAZEBO_INSTALL_PREFIX ${GAZEBO_LIBRARY_DIR0}/..)
endif()
set(GAZEBO_SHARE_DIR ${GAZEBO_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION})
set(GAZEBO_SETUP_SH ${GAZEBO_SHARE_DIR}/setup.sh)

configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/)

add_subdirectory(plugins)
add_subdirectory(models)
add_subdirectory(worlds)
