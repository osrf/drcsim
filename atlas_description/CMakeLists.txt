cmake_minimum_required(VERSION 2.8.3)
project(atlas_description)

find_package(catkin REQUIRED)

catkin_package(
)

set(standalone_name "standalone_models")

install(DIRECTORY
  urdf meshes meshes_v3 materials robots raw ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
include(${CMAKE_SOURCE_DIR}/tools/URDF_helpers.cmake)
make_standalone_models(atlas_description atlas ${standalone_name})

# Copy all the meshes and materials into standalone_models/atlas
install(DIRECTORY
  meshes materials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${standalone_name}/atlas
)
