cmake_minimum_required(VERSION 2.8.3)
project(handle_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  CableTension.msg
  Collision.msg
  Finger.msg
  HandleCollisions.msg
  HandleControl.msg
  HandleSensors.msg
  HandleSensorsCalibrated.msg
  )

generate_messages(DEPENDENCIES
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
  )
