cmake_minimum_required(VERSION 2.8.3)
project(atlas_description)

find_package(catkin REQUIRED)

catkin_package(
)

# We'll use xsltproc for XML processing.  We only need it to produce static
# model files, which we'll consider to be optional.
find_program(xsltproc xsltproc)
if(NOT xsltproc)
  message(WARNING "Couldn't find xsltproc, so won't do XML transformations.")
endif()

# For each .xacro file, run xacro on it to produce a standalone URDF file.  If
# xsltproc is available, also strip out Gazebo-specific tags (e.g, <plugin>) and
# generate a model.config.
file(GLOB urdf_xacro_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/robots "robots/*.urdf.xacro")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/static_models)
foreach(xacro ${urdf_xacro_files})
  string(REPLACE ".urdf.xacro" "" base ${xacro})
  string(REPLACE ".urdf.xacro" ".urdf" urdf ${xacro})
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base})
  if(xsltproc)
    # Convert foo.urdf.xacro -> foo.urdf.pre-xslt -> foo.urdf
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/${urdf}
                       COMMAND rosrun xacro xacro.py ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro} > ${CMAKE_CURRENT_BINARY_DIR}/${urdf}.pre-xslt
                       COMMAND ${xsltproc} ${CMAKE_SOURCE_DIR}/tools/xslt/sdf-strip-plugin.xslt ${CMAKE_CURRENT_BINARY_DIR}/${urdf}.pre-xslt > ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/${urdf}
    
                       DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro})
    # Create custom foo/model.config
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/model.config
                       COMMAND ${xsltproc} --stringparam model_name ${base} --stringparam model_sdf ${urdf} ${CMAKE_SOURCE_DIR}/tools/xslt/model.config.xslt ${CMAKE_SOURCE_DIR}/tools/xslt/model.config.xml > ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/model.config)
    add_custom_target(${urdf}_model_config_gen ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/model.config)
  else()
    # Convert foo.urdf.xacro -> foo.urdf
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/${urdf}
                       COMMAND rosrun xacro xacro.py ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro} > ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/${urdf}
                       DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro})
  endif()
  add_custom_target(${urdf}_gen ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/static_models/${base}/${urdf})
endforeach()

install(DIRECTORY
  urdf meshes meshes_v3 materials robots raw ${CMAKE_CURRENT_BINARY_DIR}/static_models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  dummy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
