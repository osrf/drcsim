# compile using xacro
set (model_basenames irobot_hand_on_box)
foreach(model_basename ${model_basenames})
  add_custom_command(
    OUTPUT ${model_basename}.urdf
    COMMAND ROS_PACKAGE_PATH=${CMAKE_SOURCE_DIR}/ros:$ENV{ROS_PACKAGE_PATH} && rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf 
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
          DESTINATION ${GAZEBO_INSTALL_PATH}/${model_basename})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.manifest.xml
          DESTINATION ${GAZEBO_INSTALL_PATH}/${model_basename}
          RENAME manifest.xml)
endforeach()

#set (files
#  irobot_hand_sandia_hand.model.xacro
#  irobot_hand.model.xacro
#)
#install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/robots/)

