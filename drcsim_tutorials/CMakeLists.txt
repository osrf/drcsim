add_subdirectory(drcsim_gazebo)
add_subdirectory(atlas_description)
add_subdirectory(irobot_hand_description)
add_subdirectory(multisense_sl_description)

# Construct the ROS_PACKAGE_PATH that we'll need, both at configure and build time.
set(ROS_PACKAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/atlas_msgs:${OSRF_COMMON_ROS_PACKAGE_PATH}:$ENV{ROS_PACKAGE_PATH})
ExternalProject_Add(atlas_msgs
                    UPDATE_COMMAND true
                    PATCH_COMMAND true
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/atlas_msgs
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/atlas_msgs
                    BUILD_COMMAND env ROS_ROOT=$ENV{ROS_ROOT} ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} make -j4
                    CMAKE_ARGS
                      -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
                      -DROS_ROOT=$ENV{ROS_ROOT}
                      -DROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
                      -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
                      -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
                      -DPLUGIN_LINK_DIRECTORIES=${CMAKE_BINARY_DIR}/plugins
                    INSTALL_COMMAND ""
                    )
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/atlas_msgs install)")

# Construct the ROS_PACKAGE_PATH that we'll need, both at configure and build time.
set(ROS_PACKAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/joint_commands_gui:${OSRF_COMMON_ROS_PACKAGE_PATH}:$ENV{ROS_PACKAGE_PATH})
ExternalProject_Add(joint_commands_gui
                    UPDATE_COMMAND true
                    PATCH_COMMAND true
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/joint_commands_gui
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/joint_commands_gui
                    BUILD_COMMAND env ROS_ROOT=$ENV{ROS_ROOT} ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} make -j4
                    CMAKE_ARGS
                      -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
                      -DROS_ROOT=$ENV{ROS_ROOT}
                      -DROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
                      -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
                      -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
                      -DPLUGIN_LINK_DIRECTORIES=${CMAKE_BINARY_DIR}/plugins
                    INSTALL_COMMAND ""
                    )
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/joint_commands_gui install)")

# Construct the ROS_PACKAGE_PATH that we'll need, both at configure and build time.
set(ROS_PACKAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/atlas_teleop:${OSRF_COMMON_ROS_PACKAGE_PATH}:$ENV{ROS_PACKAGE_PATH})
ExternalProject_Add(atlas_teleop
                    UPDATE_COMMAND true
                    PATCH_COMMAND true
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/atlas_teleop
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/atlas_teleop
                    BUILD_COMMAND env ROS_ROOT=$ENV{ROS_ROOT} ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} make -j4
                    CMAKE_ARGS
                      -DPKG_CONFIG_PATH=${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}
                      -DROS_ROOT=$ENV{ROS_ROOT}
                      -DROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DDRC_SIM_PROJECT_NAME=${PROJECT_NAME}
                      -DDRC_SIM_VERSION=${DRC_SIM_VERSION}
                      -DDRC_SIM_CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
                      -DPLUGIN_LINK_DIRECTORIES=${CMAKE_BINARY_DIR}/plugins
                    INSTALL_COMMAND ""
                    )
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/atlas_teleop install)")
