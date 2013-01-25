cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

project (drcsim)
include(ExternalProject)

set (DRC_SIM_MAJOR_VERSION 1)
set (DRC_SIM_MINOR_VERSION 4)
set (DRC_SIM_PATCH_VERSION 0)
set (DRC_SIM_VERSION ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION})
set (DRC_SIM_VERSION_FULL ${DRC_SIM_MAJOR_VERSION}.${DRC_SIM_MINOR_VERSION}.${DRC_SIM_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${DRC_SIM_VERSION_FULL}")

# Packaging configuration
set (CPACK_PACKAGE_VERSION "${DRC_SIM_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${DRC_SIM_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${DRC_SIM_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${DRC_SIM_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;TODO;/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@DRC_SIM_VERSION_FULL@")
include (CPack)

set (ROS_DISTRO "fuerte")
execute_process(COMMAND rosrun rosunit test_results_dir.py
                RESULT_VARIABLE test_results_error
                OUTPUT_VARIABLE test_results_stdout
                ERROR_VARIABLE test_results_stderr
                OUTPUT_STRIP_TRAILING_WHITESPACE)
if(test_results_error)
  message(FATAL_ERROR "Failed to determine output location for ROS tests.")
endif()
set(ROS_TEST_RESULTS_DIR ${test_results_stdout})

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

configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/)
configure_file(${CMAKE_SOURCE_DIR}/cmake/setup-unversioned.sh.in ${PROJECT_BINARY_DIR}/unversioned/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/unversioned/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/drcsim/)

add_subdirectory(gazebo_models)
add_subdirectory(plugins)
add_subdirectory(ros)
add_subdirectory(worlds)
