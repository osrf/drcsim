add_subdirectory(atlas_utils)
add_subdirectory(atlas_description)
add_subdirectory(irobot_hand_description)

add_subdirectory(drcsim_plugins)

# ExternalProject_Add(drcsim_plugins
#                     UPDATE_COMMAND true
#                     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drcsim_plugins
#                     BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/drcsim_plugins
#                     BUILD_COMMAND make -j4
#                     CMAKE_ARGS
#                       -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
#                       -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
#                       -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
#                       -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
#                       -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
#                       -DDRC_SIM_CMAKE_CURRENT_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}
#                     INSTALL_COMMAND "")
# add_dependencies(drcsim_plugins DRCVehiclePlugin)
# install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/drcsim_plugins install)")
