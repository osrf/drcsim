cmake_minimum_required(VERSION 2.8.3)
project(atlas_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
  sandia_hand_msgs
  control_msgs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  AtlasBehaviorFeedback.msg
  AtlasBehaviorManipulateFeedback.msg
  AtlasBehaviorManipulateParams.msg
  AtlasBehaviorPelvisServoParams.msg
  AtlasBehaviorStandFeedback.msg
  AtlasBehaviorStandParams.msg
  AtlasBehaviorStepData.msg
  AtlasBehaviorStepFeedback.msg
  AtlasBehaviorStepParams.msg
  AtlasBehaviorWalkFeedback.msg
  AtlasBehaviorWalkParams.msg
  AtlasCommand.msg
  AtlasPositionData.msg
  AtlasSimInterfaceCommand.msg
  AtlasSimInterfaceState.msg
  AtlasState.msg
  ControllerStatistics.msg
  ForceTorqueSensors.msg
  SynchronizationStatistics.msg
  Test.msg
  VRCScore.msg
  )

add_service_files(DIRECTORY srv FILES
  AtlasFilters.srv
  GetJointDamping.srv
  ResetControls.srv
  SetJointDamping.srv
  )

add_action_files(DIRECTORY action FILES
  WalkDemo.action
 )

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
  sandia_hand_msgs
  control_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
  sandia_hand_msgs
  message_generation
  )
