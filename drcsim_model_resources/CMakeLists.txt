cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

project (drcsim_model_resources)

#####################################
set (ATLAS_SIM_INTERFACE_3_MAJOR_VERSION 3)
set (ATLAS_SIM_INTERFACE_3_MINOR_VERSION 0)
set (ATLAS_SIM_INTERFACE_3_PATCH_VERSION 2)

set (ATLAS_SIM_INTERFACE_2_MAJOR_VERSION 2)
set (ATLAS_SIM_INTERFACE_2_MINOR_VERSION 10)
set (ATLAS_SIM_INTERFACE_2_PATCH_VERSION 2)

set (ATLAS_SIM_INTERFACE_1_MAJOR_VERSION 1)
set (ATLAS_SIM_INTERFACE_1_MINOR_VERSION 1)
set (ATLAS_SIM_INTERFACE_1_PATCH_VERSION 1)

set (ATLAS_SIM_INTERFACE_3_VERSION ${ATLAS_SIM_INTERFACE_3_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_3_MINOR_VERSION})
set (ATLAS_SIM_INTERFACE_3_VERSION_FULL ${ATLAS_SIM_INTERFACE_3_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_3_MINOR_VERSION}.${ATLAS_SIM_INTERFACE_3_PATCH_VERSION})

set (ATLAS_SIM_INTERFACE_2_VERSION ${ATLAS_SIM_INTERFACE_2_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_2_MINOR_VERSION})
set (ATLAS_SIM_INTERFACE_2_VERSION_FULL ${ATLAS_SIM_INTERFACE_2_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_2_MINOR_VERSION}.${ATLAS_SIM_INTERFACE_2_PATCH_VERSION})

set (ATLAS_SIM_INTERFACE_1_VERSION ${ATLAS_SIM_INTERFACE_1_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_1_MINOR_VERSION})
set (ATLAS_SIM_INTERFACE_1_VERSION_FULL ${ATLAS_SIM_INTERFACE_1_MAJOR_VERSION}.${ATLAS_SIM_INTERFACE_1_MINOR_VERSION}.${ATLAS_SIM_INTERFACE_1_PATCH_VERSION})

#####################################
find_package(catkin)
catkin_package(
  CFG_EXTRAS drcsim_model_resources-extras.cmake
)

# set (DRC_SIM_MAJOR_VERSION 3)
# set (DRC_SIM_MINOR_VERSION 0)
# # The patch version may have been bumped for prerelease purposes; be sure to
# # check drcsim-release/ubuntu/debian/changelog@default to determine what the
# # next patch version should be for a regular release.
# set (DRC_SIM_PATCH_VERSION 0)
# set (DRC_SIM_VERSION ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION})
# set (DRC_SIM_VERSION_FULL ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION}.${DRC_SIM_PATCH_VERSION})
# message (STATUS "${PROJECT_NAME} version ${DRC_SIM_VERSION_FULL}")

# Documentation
add_subdirectory(doc)

# If we're configuring only to build docs, stop here
if (DOC_ONLY)
  message(WARNING "Configuration was done in DOC_ONLY mode.  You can build documentation (make doc), but nothing else.")
  return()
endif()

# Packaging configuration
# set (CPACK_PACKAGE_VERSION "${DRC_SIM_VERSION_FULL}")
# set (CPACK_PACKAGE_VERSION_MAJOR "${DRC_SIM_MAJOR_VERSION}")
# set (CPACK_PACKAGE_VERSION_MINOR "${DRC_SIM_MINOR_VERSION}")
# set (CPACK_PACKAGE_VERSION_PATCH "${DRC_SIM_PATCH_VERSION}")
# list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
# list (APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;TODO;/.hg/;.hgignore;.swp$;/build/")
# set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@DRC_SIM_VERSION_FULL@")
# include (CPack)

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

#####################################
# AtlasSimInterface 3.0.x V4 and V5 shim library
include_directories(
  ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL}/include
  ${PROJECT_SOURCE_DIR}/AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL}
)

add_library(AtlasSimInterface3 SHARED AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL}/src/AtlasSimInterface.cc)
set_target_properties(AtlasSimInterface3 PROPERTIES SOVERSION ${ATLAS_SIM_INTERFACE_3_VERSION_FULL})
install (TARGETS AtlasSimInterface3 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_3_VERSION_FULL} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#####################################
# AtlasSimInterface 2.10.2 V3 shim library
include_directories(
  ${PROJECT_SOURCE_DIR}/AtlasSimInterface_2.10.2/include
  ${PROJECT_SOURCE_DIR}/AtlasSimInterface_2.10.2/include/AtlasSimInterface_2.10.2
)

add_library(AtlasSimInterface2 SHARED AtlasSimInterface_2.10.2/src/AtlasSimInterface.cc)
set_target_properties(AtlasSimInterface2 PROPERTIES SOVERSION ${ATLAS_SIM_INTERFACE_2_VERSION_FULL})
install (TARGETS AtlasSimInterface2 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_2_VERSION_FULL}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_2_VERSION_FULL} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#####################################
# install AtlasSimInterface 1.1.1 Binary

# libAtlasSimInterface.so.1.1.1 provided by BDI for 64bit systems
# on 32bit systems use shim library
if (NOT DEFINED ATLAS_SIMINTERFACE_1_BINARY_EXISTS)
  if (UNIX)
    # Check for 64bits machines
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(ATLAS_SIMINTERFACE_1_BINARY_EXISTS True)
    else()
      set(ATLAS_SIMINTERFACE_1_BINARY_EXISTS False)
      message(FATAL_ERROR "WARNING: AtlasSIMINTERFACE binary will *NOT* be used in 32 bits systems. "
                          "If you want to use DRCSim without the AtlasSimInterface binary interface, "
                          "run cmake with -DUSE_ATLASSIMINTERFACE_1_BINARY_EXISTS:BOOL=False")
    endif()
  else()
    message(STATUS "Warning: system check for 32/64 bit is not implemented for")
    message(STATUS "Warning: your system, assuming 64bit machine. Please use")
    message(STATUS "Warning: please use ATLAS_SIMINTERFACE_1_BINARY_EXISTS flag if you want")
    message(STATUS "Warning: not to use 64 bits AtlasSIMINTERFACE library")
    set(ATLAS_SIMINTERFACE_1_BINARY_EXISTS True)
  endif()
else()
  message(STATUS "compiler definition ATLAS_SIMINTERFACE_1_BINARY_EXISTS defined by user, "
                 "going forward assuming libAtlasSimInterface.so 1.1.1 binaries exist in "
                 "drcsim_model_resources/AtlasSimInterface_1.1.1/lib64")
endif()

# set(ATLAS_SIMINTERFACE_1_BINARY_EXISTS False) #testing shim for AtlasSimInterface 1

if (NOT ATLAS_SIMINTERFACE_1_BINARY_EXISTS)
  message(STATUS "============================================\n"
                 "Using AtlasSimInterface 1.1.1 shim interface\n"
                 "============================================")
  include_directories(
    ${PROJECT_SOURCE_DIR}/AtlasSimInterface_1.1.1/include/AtlasSimInterface_1.1.1
  )
  add_library(AtlasSimInterface SHARED AtlasSimInterface_1.1.1/src/AtlasSimInterface.cc)
  set_target_properties(AtlasSimInterface PROPERTIES SOVERSION ${ATLAS_SIM_INTERFACE_1_VERSION_FULL})
  install (TARGETS AtlasSimInterface DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  message(STATUS "=======================================================\n"
                 "Installing AtlasSimInterface 1.1.1 BDI binary interface\n"
                 "=======================================================")
  install (FILES AtlasSimInterface_${ATLAS_SIM_INTERFACE_1_VERSION_FULL}/lib64/libAtlasSimInterface.so.${ATLAS_SIM_INTERFACE_1_VERSION_FULL}
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  install (FILES AtlasSimInterface_${ATLAS_SIM_INTERFACE_1_VERSION_FULL}/lib64/libAtlasSimInterface.so
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

# install headers
install (DIRECTORY AtlasSimInterface_${ATLAS_SIM_INTERFACE_1_VERSION_FULL}/include/AtlasSimInterface_${ATLAS_SIM_INTERFACE_1_VERSION_FULL}
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#####################################
# Configure the setup script
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install also a copy of setup.sh in /usr/share/drcsim/setup.sh for backwards compatibility
# Avoid to use symlinks for portability reasons. Copy the file twice.
if(UNIX AND NOT APPLE)
  # Hack: only install setup.sh in /usr/share/drcsim if destination if /opt/ros.
  # This will cover the cases of system installations from source and
  # debs and respects catkin workspaces. Installing outside of
  # /opt/ros is mostly to be broken for system instalations.
  string(SUBSTRING "${CMAKE_INSTALL_PREFIX}" 0 8 PREFIX_START)
  if (${PREFIX_START} STREQUAL "/opt/ros")
    install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION /usr/share/drcsim)
  else()
    # In local installations, replicate the previous share/drcsim/setup.sh
    install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drcsim)
  endif()
endif()

configure_file(${PROJECT_SOURCE_DIR}/cmake/rosinstall.in ${PROJECT_BINARY_DIR}/.rosinstall @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/.rosinstall DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}/)
# install(FILES ${PROJECT_BINARY_DIR}/.rosinstall DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}-${DRC_SIM_VERSION}/)
install(PROGRAMS ${PROJECT_SOURCE_DIR}/tools/mkqual.bash DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)


# install gazebo model resource files
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
