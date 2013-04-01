# compile robot from xacro

# note the hand integrated needs ros package sandia_hand_description installed by sandia-hand to be in ROS_PACKAGE_PATH already
# (e.g. export ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share/sandia-hand/ros:${ROS_PACKAGE_PATH} or similar)

# this package and irobot hand package also need to to be in ROS_PACKAGE_PATH
# compile using xacro

if (HAVE_SANDIA_HAND)
  set (model_basenames atlas atlas_irobot_hands atlas_sandia_hands)
else()
  set (model_basenames atlas atlas_irobot_hands)
endif()


foreach(model_basename ${model_basenames})
  add_custom_command(
    OUTPUT ${model_basename}.urdf
    COMMAND export ROS_PACKAGE_PATH=${SANDIA_HAND_ROS_PACKAGE_PATH}:${CMAKE_SOURCE_DIR}/ros:$ENV{ROS_PACKAGE_PATH} && rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf 
    DEPENDS ${model_basename}.urdf.xacro)
  add_custom_target(${model_basename}_urdf ALL DEPENDS ${model_basename}.urdf)

  add_custom_command(
    OUTPUT ${model_basename}.model.config
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_manifest_generator.sh ${model_basename} > ${model_basename}.model.config
    DEPENDS gazebo_manifest_generator.sh)
  add_custom_target(${model_basename}_manifest ALL DEPENDS ${model_basename}.model.config)
    
  #install(FILES ${model_basename}.urdf.xacro
  #        DESTINATION ${ROS_PKG_INSTALL_PATH}/robots/)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf
          DESTINATION ${GAZEBO_INSTALL_PATH}/${model_basename})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.model.config
          DESTINATION ${GAZEBO_INSTALL_PATH}/${model_basename}
          RENAME model.config)
  # In gazebo-1.4, the model manifests were renamed from manifest.xml to model.config
  # For backwards compatibility, we will continue to install a duplicate manifest.xml
  # alongside the model.config
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.model.config
          DESTINATION ${GAZEBO_INSTALL_PATH}/${model_basename}
          RENAME manifest.xml)
endforeach()

