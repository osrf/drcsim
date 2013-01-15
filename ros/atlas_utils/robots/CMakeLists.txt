# compile robot from xacro

# note the hand integrated needs ros package sandia_hand_description installed by sandia-hand to be in ROS_PACKAGE_PATH already
# (e.g. export ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share/sandia-hand/ros:${ROS_PACKAGE_PATH} or similar)

# this package and irobot hand package also need to to be in ROS_PACKAGE_PATH
# compile using xacro

if (HAVE_SANDIA_HAND)
  set(ENV{ROS_PACKAGE_PATH} ${_ROS_PACKAGE_PATH}:${CMAKE_SOURCE_DIR}/ros:${ROS_PACKAGE_PATH})
  set (model_basenames atlas atlas_sandia_hands)
else()
  set(ENV{ROS_PACKAGE_PATH}                      ${CMAKE_SOURCE_DIR}/ros:${ROS_PACKAGE_PATH})
  set (model_basenames atlas)
endif()

foreach(model_basename ${model_basenames})
  add_custom_command(
    OUTPUT ${model_basename}.urdf
    COMMAND rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${model_basename}.urdf 
    DEPENDS ${model_basename}.urdf.xacro)
  add_custom_target(${model_basename}_urdf ALL DEPENDS ${model_basename}.urdf)

  add_custom_command(
    OUTPUT ${model_basename}.manifest.xml
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_manifest_generator.sh ${model_basename} > ${model_basename}.manifest.xml
    DEPENDS gazebo_manifest_generator.sh)
  add_custom_target(${model_basename}_manifest ALL DEPENDS ${model_basename}.manifest.xml)
    
  install(FILES ${model_basename}.urdf.xacro
          DESTINATION ${ROS_PKG_INSTALL_PATH}/robots/)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.manifest.xml
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename}
          RENAME manifest.xml)
endforeach()

#set (files
#  atlas_sandia_hand.model.xacro
#  atlas.model.xacro
#)
#install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/robots/)

