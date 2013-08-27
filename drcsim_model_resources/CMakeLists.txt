cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

project (drcsim_model_resources)
catkin_package()

set (DRC_SIM_MAJOR_VERSION 2)
set (DRC_SIM_MINOR_VERSION 8)
# The patch version may have been bumped for prerelease purposes; be sure to
# check drcsim-release/ubuntu/debian/changelog@default to determine what the
# next patch version should be for a regular release.
set (DRC_SIM_PATCH_VERSION 0)
set (DRC_SIM_VERSION ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION})
set (DRC_SIM_VERSION_FULL ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION}.${DRC_SIM_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${DRC_SIM_VERSION_FULL}")

# Documentation
add_subdirectory(doc)

# If we're configuring only to build docs, stop here
if (DOC_ONLY)
  message(WARNING "Configuration was done in DOC_ONLY mode.  You can build documentation (make doc), but nothing else.")
  return()
endif()

if ("${CATKIN_BUILD_BINARY_PACKAGE}" STREQUAL "1")
  set (CATKIN_BUILD_BINARY_PACKAGE_STRING "-DCATKIN_BUILD_BINARY_PACKAGE=1")
else()
  set (CATKIN_BUILD_BINARY_PACKAGE_STRING "")
endif()

# Packaging configuration
set (CPACK_PACKAGE_VERSION "${DRC_SIM_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${DRC_SIM_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${DRC_SIM_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${DRC_SIM_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;TODO;/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@DRC_SIM_VERSION_FULL@")
include (CPack)

# Infer ROS distro from the user's environment, allowing the user to override
# manually.
if (NOT ROS_DISTRO)
  find_program(ROSVERSION_PATH rosversion)
  if(NOT ROSVERSION_PATH)
    message(FATAL_ERROR "Can't find path to rosversion executable, which is needed to determine which ROS distro you're using.  Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?  You can manually override this check by passing `-DROS_DISTRO=<distro>` to cmake (but that really should not be necessary).")
  endif()
  execute_process(COMMAND ${ROSVERSION_PATH} -d
                  OUTPUT_VARIABLE ROS_DISTRO
                  RESULT_VARIABLE rosversion_error
                  ERROR_VARIABLE rosversion_stderr
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(rosversion_error)
    message(FATAL_ERROR "`rosversion -d` failed. stdout: ${ROS_DISTRO}.  stderr: ${rosversion_stderr}.")
  endif()
endif()
message(STATUS "Building against ROS distro ${ROS_DISTRO}, expected to be installed in /opt/ros/${ROS_DISTRO}")

# rosversion can appear in /usr/bin and will return a valid result even if the
# user hasn't sourced a ROS setup.sh.  So we'll further check that ROS_ROOT is
# set and bail if not.
if("$ENV{ROS_ROOT}" STREQUAL "")
  message(FATAL_ERROR "ROS_ROOT is not set. Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?")
endif()

set (INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}-${DRC_SIM_VERSION}/${PROJECT_NAME}") 
set (LIB_INSTALL_DIR "lib" CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)") 
set (BIN_INSTALL_DIR "bin" CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)") 

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release RelWithDebInfo" FORCE)
  message(STATUS "Build type not selected: Release selected by default")
endif (NOT CMAKE_BUILD_TYPE)

# find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo>=1.3.0 REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# find osrf-common
find_package(osrf-common)
if (NOT osrf-common_FOUND)
  message(STATUS "osrf-common not found, not building atlas with joint controllers.")
  set(HAVE_OSRF_COMMON false)
else ()
  message(STATUS "osrf-common found [${OSRF_COMMON_ROS_PACKAGE_PATH}], building atlas with joint controllers.")
  set(HAVE_OSRF_COMMON true)
endif ()

# find sandia-hand
find_package(sandia-hand)
if (NOT sandia-hand_FOUND)
  message(STATUS "sandia-hand not found, not building atlas with sandia hand")
  set(HAVE_SANDIA_HAND false)
else ()
  message(STATUS "sandia-hand found [${SANDIA_HAND_ROS_PACKAGE_PATH}], building atlas with sandia hand")
  set(HAVE_SANDIA_HAND true)
endif ()

#####################################
# Configure the setup script

configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/)

# Also install the setup.sh in an unversioned location
install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drcsim/)
configure_file(${PROJECT_SOURCE_DIR}/cmake/rosinstall.in ${PROJECT_BINARY_DIR}/.rosinstall @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/.rosinstall DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/)
install(PROGRAMS ${PROJECT_SOURCE_DIR}/tools/mkqual.bash DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)

#####################################
set (ATLAS_SIM_INTERFACE_MAJOR_VERSION 1)
set (ATLAS_SIM_INTERFACE_MINOR_VERSION 1)
set (ATLAS_SIM_INTERFACE_PATCH_VERSION 1)
set (ATLAS_SIM_INTERFACE_VERSION ${ATLAS_SIM_INTERFACE_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_MINOR_VERSION})
set (ATLAS_SIM_INTERFACE_VERSION_FULL ${ATLAS_SIM_INTERFACE_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_MINOR_VERSION}.${ATLAS_SIM_INTERFACE_PATCH_VERSION})

#message(status "------------------\n\n\n\n  COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL} && cp ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/lib64/libAtlasSimInterface.so.${ATLAS_SIM_INTERFACE_VERSION_FULL} ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/libAtlasSimInterface.so \n\n\n----------------------\n")

# install AtlasSimInterface Binary library
add_custom_command(
  OUTPUT ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/libAtlasSimInterface.so
  COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL} && cp ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/lib64/libAtlasSimInterface.so.${ATLAS_SIM_INTERFACE_VERSION_FULL} ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/libAtlasSimInterface.so
)
add_custom_target(AtlasSimInterface_lib DEPENDS ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/libAtlasSimInterface.so)

# install AtlasSimInterface includes
add_custom_command(
  OUTPUT ${CMAKE_INSTALL_PREFIX}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
  COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/include && cp -r ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/include ${CMAKE_INSTALL_PREFIX}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
)
add_custom_target(AtlasSimInterface_include DEPENDS ${CMAKE_INSTALL_PREFIX}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})

# install AtlasSimInterfac doc 
add_custom_command(
  OUTPUT ${CMAKE_INSTALL_PREFIX}/share/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
  COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share && cp -r ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/doc ${CMAKE_INSTALL_PREFIX}/share/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
)
add_custom_target(AtlasSimInterface_doc DEPENDS ${CMAKE_INSTALL_PREFIX}/share/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})

# install AtlasSimInterface Binary
# install (FILES AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/lib64/libAtlasSimInterface.so.${ATLAS_SIM_INTERFACE_VERSION_FULL}
#          DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}
#          RENAME libAtlasSimInterface.so)
# install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/include
#          DESTINATION ${CMAKE_INSTALL_PREFIX}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})
# install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL}/doc
#          DESTINATION ${CMAKE_INSTALL_PREFIX}/share/AtlasSimInterface_${ATLAS_SIM_INTERFACE_VERSION_FULL})

add_subdirectory(media)
add_subdirectory(gazebo_models)
add_subdirectory(worlds)

########### Add uninstall target ###############
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${PROJECT_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")
