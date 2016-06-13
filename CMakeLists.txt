# This file allows the developer to skip the step of embedding this repo into a
# catkin workspace (by, e.g., calling catkin_init_workspace).  This repo can
# still be embedded in a catkin workspace, in which case, this file will not be
# processed (because there's no package.xml in this directory).
cmake_minimum_required(VERSION 2.8)

find_package(catkin REQUIRED)

include(${catkin_EXTRAS_DIR}/toplevel.cmake)

project (drcsim)
set (DRC_SIM_MAJOR_VERSION 7)
set (DRC_SIM_MINOR_VERSION 0)
#The patch version may have been bumped for prerelease purposes; be sure to
# check drcsim-release/ubuntu/debian/changelog@default to determine what the
# next patch version should be for a regular release.
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
