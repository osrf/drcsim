add_subdirectory(atlas_utils)
add_subdirectory(atlas_description)
add_subdirectory(irobot_hand_description)
add_subdirectory(multisense_sl_description)
#add_subdirectory(atlas_msgs)

# set(ENV{ROS_PACKAGE_PATH} ${CMAKE_SOURCE_DIR}/ros:$ENV{ROS_PACKAGE_PATH})
# ExternalProject_Add(drcsim_plugins
#                     #UPDATE_COMMAND true
#                     #PATCH_COMMAND true
#                     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drcsim_plugins
#                     BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/drcsim_plugins
#                     BUILD_COMMAND make -j4
#                     CMAKE_ARGS
#                       -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
#                       -DROS_PACKAGE_PATH=$ENV{ROS_PACKAGE_PATH}
#                       -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
#                       -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
#                       -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
#                       -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
#                       -DDRC_SIM_CMAKE_CURRENT_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}
#                     INSTALL_COMMAND "")
# add_dependencies(drcsim_plugins DRCVehiclePlugin)
# install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/drcsim_plugins install)")

set(ENV{ROS_PACKAGE_PATH} ${CMAKE_SOURCE_DIR}/ros:$ENV{ROS_PACKAGE_PATH})
set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
message(status "\n\n\n PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}\n\n\n")
ExternalProject_Add(atlas_msgs
                    UPDATE_COMMAND true
                    PATCH_COMMAND true
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/atlas_msgs
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/atlas_msgs
                    BUILD_COMMAND make -j4
                    CMAKE_ARGS
                      -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
                      -DROS_PACKAGE_PATH=${OSRF_COMMON_ROS_PACKAGE_PATH}:$ENV{ROS_PACKAGE_PATH}
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
                      -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
                      -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
                      -DPLUGIN_LINK_DIRECTORIES=${CMAKE_BINARY_DIR}/plugins
                    INSTALL_COMMAND ""
                    )
#add_dependencies(drcsim_plugins atlas_msgs)
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/atlas_msgs install)")
