cmake_minimum_required(VERSION 2.8.3)
project(atlas_description)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY
  urdf meshes meshes_v3 materials robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  dummy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
