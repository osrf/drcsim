cmake_minimum_required(VERSION 2.8.3)
project(irobot_hand_description)

# Used by launch
set(ROS_PACKAGE_NAME irobot_hand_description)

find_package(catkin REQUIRED)

catkin_package(
)

set(standalone_name "drcsim_gazebo_standalone_models")

install(DIRECTORY urdf meshes robots launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share
)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  dummy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# In addition to the usual installation, which happens above, we'll
# also do some special processing and installation to provide standalone
# versions of the models that can be used without ROS (or even Gazebo).
include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/URDF_helpers.cmake)
make_standalone_models(irobot_hand_description irobot_hand_description ${standalone_name})

# Copy all the meshes and materials into drcsim_standalone_models/atlas_description
install(DIRECTORY
  meshes
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${standalone_name}/irobot_hand_description
)
