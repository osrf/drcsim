cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo_ros_plugins)
# set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS drcsim_model_resources drcsim_gazebo_plugins osrf_msgs sandia_hand_msgs actionlib roscpp std_msgs sensor_msgs geometry_msgs trajectory_msgs control_msgs image_transport tf actionlib atlas_msgs) # gazebo_plugins)

  
# find_package(Boost REQUIRED COMPONENTS system)
# catkin_python_setup()
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drcsim_gazebo_ros_plugins
#  CATKIN_DEPENDS drcsim_gazebo_plugins # gazebo_plugins
#  DEPENDS system_lib
)

message(status "\n\n\n${catkin_INCLUDE_DIRS}\n\n")

find_package(gazebo REQUIRED)
# \todo: special hack to make gazebo flags behave.  can we remove these?
set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a cpp library
add_library(VRCPlugin src/VRCPlugin.cpp)
add_dependencies(VRCPlugin atlas_msgs_gencpp)
set_target_properties(VRCPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(VRCPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(VRCPlugin ${catkin_LIBRARIES})

add_library(SandiaHandPlugin src/SandiaHandPlugin.cpp)
set_target_properties(SandiaHandPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(SandiaHandPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(SandiaHandPlugin ${catkin_LIBRARIES})
add_dependencies(SandiaHandPlugin atlas_msgs_gencpp)

add_library(MultiSenseSLPlugin src/MultiSenseSLPlugin.cpp)
set_target_properties(MultiSenseSLPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(MultiSenseSLPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(MultiSenseSLPlugin ${catkin_LIBRARIES})


add_library(DRCVehicleROSPlugin src/DRCVehicleROSPlugin.cpp)
set_target_properties(DRCVehicleROSPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(DRCVehicleROSPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(DRCVehicleROSPlugin ${catkin_LIBRARIES})
add_dependencies(DRCVehicleROSPlugin DRCVehiclePlugin)

add_library(ContactModelPlugin src/ContactModelPlugin.cpp)
set_target_properties(ContactModelPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(ContactModelPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(ContactModelPlugin ${catkin_LIBRARIES})

add_library(AtlasPlugin src/AtlasPlugin.cpp)
set_target_properties(AtlasPlugin PROPERTIES LINK_FLAGS "${ld_flags} -L${CMAKE_INSTALL_PREFIX}/lib/AtlasSimInterface_1.1.1")
set_target_properties(AtlasPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags} -I${CMAKE_INSTALL_PREFIX}/include")
target_link_libraries(AtlasPlugin ${catkin_LIBRARIES} AtlasSimInterface)
add_dependencies(AtlasPlugin atlas_msgs_gencpp AtlasSimInterface_include AtlasSimInterface_lib)

add_library(VRCScoringPlugin src/VRCScoringPlugin.cc)
set_target_properties(VRCScoringPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(VRCScoringPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(VRCScoringPlugin ${catkin_LIBRARIES})
add_dependencies(VRCScoringPlugin atlas_msgs_gencpp)

add_library(test_ros_plugin src/test_ros_plugin.cc)
set_target_properties(test_ros_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(test_ros_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(test_ros_plugin ${catkin_LIBRARIES})

### executables, utilities for debugging atlas simulation, should not depend on gazebo
add_executable(pub_joint_trajectory_test src/pub_joint_trajectory_test.cpp)
set_target_properties(pub_joint_trajectory_test PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(pub_joint_trajectory_test PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(pub_joint_trajectory_test ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pub_joint_states src/pub_joint_states.cpp)
target_link_libraries(pub_joint_states ${catkin_LIBRARIES})
add_dependencies(pub_joint_states atlas_msgs_gencpp)

add_executable(pub_joint_commands src/pub_joint_commands.cpp)
set_target_properties(pub_joint_commands PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(pub_joint_commands PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(pub_joint_commands ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pub_joint_commands osrf_msgs_gencpp)

add_executable(pub_atlas_state src/pub_atlas_state.cpp)
target_link_libraries(pub_atlas_state ${catkin_LIBRARIES})
add_dependencies(pub_atlas_state atlas_msgs_gencpp)

add_executable(pub_atlas_command_fast src/pub_atlas_command_fast.cpp)
target_link_libraries(pub_atlas_command_fast ${catkin_LIBRARIES})
add_dependencies(pub_atlas_command_fast atlas_msgs_gencpp)

add_executable(pub_atlas_command src/pub_atlas_command.cpp)
target_link_libraries(pub_atlas_command ${catkin_LIBRARIES})
add_dependencies(pub_atlas_command atlas_msgs_gencpp)

add_executable(gz_model_teleport src/gz_model_teleport.cpp)
set_target_properties(gz_model_teleport PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gz_model_teleport PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gz_model_teleport ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

## example actionlib implementation
add_executable(actionlib_server src/actionlib_server.cpp)
target_link_libraries(actionlib_server ${catkin_LIBRARIES})
add_dependencies(actionlib_server atlas_msgs_gencpp) # or atlas_msgs_generate_messages_cpp)





##### these are repeats of gazebo_ros_pkgs/gazebo_plugins,
##### but we have to repeat ther here because gazebo_ros_pkgs is
##### released only in hydro, not in groovy/fuerte
add_library(MultiCameraPlugin use_gazebo_ros_pkgs/MultiCameraPlugin.cc)
set_target_properties(MultiCameraPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(MultiCameraPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(MultiCameraPlugin ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_camera_utils use_gazebo_ros_pkgs/gazebo_ros_camera_utils.cpp)
set_target_properties(gazebo_ros_camera_utils PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera_utils PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera_utils CameraPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_camera use_gazebo_ros_pkgs/gazebo_ros_camera.cpp)
set_target_properties(gazebo_ros_camera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_p3d use_gazebo_ros_pkgs/gazebo_ros_p3d.cpp)
set_target_properties(gazebo_ros_p3d PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_p3d PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_p3d ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_multicamera use_gazebo_ros_pkgs/gazebo_ros_multicamera.cpp)
set_target_properties(gazebo_ros_multicamera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_multicamera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_multicamera gazebo_ros_camera_utils MultiCameraPlugin ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_laser use_gazebo_ros_pkgs/gazebo_ros_laser.cpp)
set_target_properties(gazebo_ros_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_laser ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_joint_trajectory use_gazebo_ros_pkgs/gazebo_ros_joint_trajectory.cpp)
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_joint_trajectory ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_imu use_gazebo_ros_pkgs/gazebo_ros_imu.cpp)
set_target_properties(gazebo_ros_imu PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_imu PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_imu ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_gpu_laser use_gazebo_ros_pkgs/gazebo_ros_gpu_laser.cpp)
set_target_properties(gazebo_ros_gpu_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_gpu_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_gpu_laser ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} GpuRayPlugin)

add_library(gazebo_ros_force use_gazebo_ros_pkgs/gazebo_ros_force.cpp)
Set_target_properties(gazebo_ros_force PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_force PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_force ${GAZEBO_LIBRARIES})

#add_library(gazebo_ros_controller_manager use_gazebo_ros_pkgs/gazebo_ros_controller_manager.cpp)
#set_target_properties(gazebo_ros_controller_manager PROPERTIES LINK_FLAGS "${ld_flags}")
#set_target_properties(gazebo_ros_controller_manager PROPERTIES COMPILE_FLAGS "${cxx_flags}")
#target_link_libraries(gazebo_ros_controller_manager ${GAZEBO_LIBRARIES})


### this should be the same as gazebo_ros_api plugin in gazebo_ros_pkgs/gazebo_ros
### without all the refernces to gazebo_msgs.  This is because we don't want to
### rebuild gazebo_msgs here.
add_library(gazebo_ros_api_plugin use_gazebo_ros_pkgs/gazebo_ros_api_plugin.cpp)
set_target_properties(gazebo_ros_api_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_api_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_api_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


#############
## Install ##
#############
install(TARGETS VRCPlugin SandiaHandPlugin MultiSenseSLPlugin DRCVehicleROSPlugin ContactModelPlugin AtlasPlugin VRCScoringPlugin test_ros_plugin MultiCameraPlugin gazebo_ros_camera_utils gazebo_ros_camera gazebo_ros_p3d gazebo_ros_multicamera gazebo_ros_laser gazebo_ros_joint_trajectory gazebo_ros_imu gazebo_ros_gpu_laser gazebo_ros_force gazebo_ros_api_plugin pub_joint_trajectory_test pub_joint_states pub_joint_commands pub_atlas_state pub_atlas_command_fast pub_atlas_command gz_model_teleport actionlib_server # gazebo_ros_controller_manager 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/
  # DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}-${drcsim_gazebo_ros_plugins_VERSION}/plugins/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/use_gazebo_ros_pkg/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
