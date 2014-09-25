# Helper macro to do some URDF/XACRO processing.
# Params:
#   old_model_name, new_model_name: we'll replace:
#       <mesh filename="package://old_model_name/..." .../>
#     with:
#      <mesh filename="package://new_model_name/..." .../>
#   standalone_name: name of directory to contains the standalone models;
#     usually "standalone_models"
macro(make_standalone_models old_model_name new_model_name standalone_name)
  # We'll use xsltproc for XML processing.  We only need it to produce static
  # model files, which we'll consider to be optional.
  find_program(xsltproc xsltproc)
  if(NOT xsltproc)
    message(WARNING "Couldn't find xsltproc, so won't do XML transformations and will skip building of gazebo standalone models.")
  endif()
  
  # For each .xacro file, run xacro on it to produce a standalone URDF file.  If
  # xsltproc is available, also strip out Gazebo-specific tags (e.g, <plugin>) and
  # generate a model.config.
  file(GLOB urdf_xacro_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/robots "robots/*.urdf.xacro")
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name})
  foreach(xacro ${urdf_xacro_files})
    string(REPLACE ".urdf.xacro" "" base ${xacro})
    string(REPLACE ".urdf.xacro" ".urdf" urdf ${xacro})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base})
    if(xsltproc)
      # Convert foo.urdf.xacro -> foo.urdf.pre-xslt -> foo.urdf
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/${urdf}
                         COMMAND ROS_PACKAGE_PATH=${CMAKE_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro.py ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro} > ${CMAKE_CURRENT_BINARY_DIR}/${urdf}.pre-xslt
                         COMMAND ${xsltproc} --stringparam old_model_name ${old_model_name} --stringparam new_model_name ${new_model_name} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/xslt/sdf-make-standalone-model.xslt ${CMAKE_CURRENT_BINARY_DIR}/${urdf}.pre-xslt > ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/${urdf}
                         DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/xslt/sdf-make-standalone-model.xslt)
      # Create custom foo/model.config
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/model.config
                         COMMAND ${xsltproc} --stringparam model_name ${base} --stringparam model_sdf ${urdf} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/xslt/model.config.xslt ${CMAKE_CURRENT_SOURCE_DIR}/../tools/xslt/model.config.xml > ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/model.config)
      add_custom_target(${urdf}_model_config_gen ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/model.config)
    else()
      # Convert foo.urdf.xacro -> foo.urdf
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/${urdf}
                         COMMAND ROS_PACKAGE_PATH=${CMAKE_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro.py ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro} > ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/${urdf}
                         DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro})
    endif()
    add_custom_target(${urdf}_gen ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}/${base}/${urdf})
    add_dependencies(${urdf}_gen ${CMAKE_CURRENT_SOURCE_DIR}/robots/${xacro})
  endforeach()
endmacro()
