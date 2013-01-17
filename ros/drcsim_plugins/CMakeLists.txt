cmake_minimum_required(VERSION 2.4.6)
# set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ENV{ROS_PACKAGE_PATH} ${CMAKE_SOURCE_DIR}/ros/drcsim_plugins:$ENV{ROS_PACKAGE_PATH})

# save project name, paths
set(SAVE_PROJECT_NAME ${PROJECT_NAME})

# message(status "\n\n\n ${PKG_CONFIG_PATH} \n\n\n")
set(ROSBUILD_DONT_REDEFINE_PROJECT 1)
project(drcsim_plugins)
rosbuild_init()

# recover project name
#set(PROJECT_NAME ${SAVE_PROJECT_NAME})

include_directories(
  ${CMAKE_SOURCE_DIR}/plugins
)

#link_directories(${DRC_SIM_CMAKE_CURRENT_BINARY_DIR}/plugins)

# find gazebo include (FindPkgConfig)
# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(GAZEBO gazebo)
# else()
#   message(FATAL_ERROR "pkg-config is required; please install it")
# endif()
#
# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
INSTALL(CODE "EXECUTE_PROCESS(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/${SAVE_PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)")

rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

# needs pr2_controller_manager
rosbuild_add_library(gazebo_ros_controller_manager gazebo_ros_controller_manager.cpp)
rosbuild_link_boost(gazebo_ros_controller_manager thread)

# needs roscpp, trajectory_msgs
rosbuild_add_executable(pub_joint_trajectory_test pub_joint_trajectory_test.cpp)
rosbuild_add_executable(pub_trajectory_controller_test pub_trajectory_controller_test.cpp)

# needs std_msgs, sensor_msgs, geometry_msgs, image_transport tf, roscpp, trajectory_msgs
rosbuild_add_library(gazebo_ros_camera_utils gazebo_ros_camera_utils.cpp)
target_link_libraries(gazebo_ros_camera_utils CameraPlugin)
rosbuild_add_library(gazebo_ros_laser gazebo_ros_laser.cpp)
target_link_libraries(gazebo_ros_laser RayPlugin)
rosbuild_add_library(gazebo_ros_imu gazebo_ros_imu.cpp)
rosbuild_add_library(gazebo_ros_camera gazebo_ros_camera.cpp)
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils)
rosbuild_add_library(gazebo_ros_joint_trajectory gazebo_ros_joint_trajectory.cpp)
rosbuild_add_library(gazebo_ros_api_plugin gazebo_ros_api_plugin.cpp)
rosbuild_add_library(gazebo_ros_p3d gazebo_ros_p3d.cpp)

rosbuild_add_library(MultiSenseSLPlugin MultiSenseSLPlugin.cpp)
rosbuild_add_library(AtlasPlugin AtlasPlugin.cpp)

# needs actionlib, control_msgs
rosbuild_add_library(VRCPlugin VRCPlugin.cpp)
target_link_libraries(VRCPlugin)

# needs DRCVehiclePlugin from non-ros plugins
rosbuild_add_library(DRCVehicleROSPlugin DRCVehicleROSPlugin.cpp)
target_link_libraries(DRCVehicleROSPlugin DRCVehiclePlugin)
#target_link_libraries(DRCVehicleROSPlugin :../../plugins/libDRCVehiclePlugin.so)



#   get_rospack_flags(pr2_controller_manager)
#   include_directories(${pr2_controller_manager_INCLUDE_DIRS})
#   link_directories(${pr2_controller_manager_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${pr2_controller_manager_LIBRARY_DIRS}")
#   add_library(gazebo_ros_controller_manager SHARED gazebo_ros_controller_manager.cpp)
#   set_target_properties(gazebo_ros_controller_manager PROPERTIES COMPILE_FLAGS "${pr2_controller_manager_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_controller_manager PROPERTIES LINK_FLAGS "${pr2_controller_manager_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_controller_manager ${pr2_controller_manager_LIBRARIES})
#   install (TARGETS gazebo_ros_controller_manager DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#   get_rospack_flags(trajectory_msgs)
#   get_rospack_flags(roscpp)
#   include_directories(${roscpp_INCLUDE_DIRS} ${trajectory_msgs_INCLUDE_DIRS})
#   link_directories(${roscpp_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS}")
#   add_executable(pub_joint_trajectory_test pub_joint_trajectory_test.cpp)
#   set_target_properties(pub_joint_trajectory_test PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER} ${trajectory_msgs_CFLAGS_OTHER}")
#   set_target_properties(pub_joint_trajectory_test PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER} ${trajectory_msgs_LDFLAGS_OTHER}")
#   target_link_libraries(pub_joint_trajectory_test ${roscpp_LIBRARIES} ${trajectory_msgs_LIBRARIES})
#   install (TARGETS pub_joint_trajectory_test DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#   add_executable(pub_trajectory_controller_test pub_trajectory_controller_test.cpp)
#   set_target_properties(pub_trajectory_controller_test PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER} ${trajectory_msgs_CFLAGS_OTHER}")
#   set_target_properties(pub_trajectory_controller_test PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER} ${trajectory_msgs_LDFLAGS_OTHER}")
#   target_link_libraries(pub_trajectory_controller_test ${roscpp_LIBRARIES} ${trajectory_msgs_LIBRARIES})
#   install (TARGETS pub_trajectory_controller_test DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#   get_rospack_flags(std_msgs)
#   get_rospack_flags(sensor_msgs)
#   get_rospack_flags(geometry_msgs)
#   get_rospack_flags(image_transport)
#   get_rospack_flags(tf)
#   include_directories(${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(gazebo_ros_camera_utils SHARED gazebo_ros_camera_utils.cpp)
#   set_target_properties(gazebo_ros_camera_utils PROPERTIES COMPILE_FLAGS "${std_msgs_CFLAGS_OTHER} ${sensor_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_camera_utils PROPERTIES LINK_FLAGS "${std_msgs_LDFLAGS_OTHER} ${sensor_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_camera_utils ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES} CameraPlugin)
#   install (TARGETS gazebo_ros_camera_utils DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   include_directories(${sensor_msgs_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${sensor_msgs_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${sensor_msgs_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(gazebo_ros_laser SHARED gazebo_ros_laser.cpp)
#   set_target_properties(gazebo_ros_laser PROPERTIES COMPILE_FLAGS "${sensor_msgs_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_laser PROPERTIES LINK_FLAGS "${sensor_msgs_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_laser ${sensor_msgs_LIBRARIES} ${tf_LIBRARIES} RayPlugin)
#   install (TARGETS gazebo_ros_laser DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   include_directories(${sensor_msgs_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS})
#   link_directories(${sensor_msgs_LIBRARY_DIRS} ${std_srvs_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${sensor_msgs_LIBRARY_DIRS} ${std_srvs_LIBRARY_DIRS}")
#   add_library(gazebo_ros_imu SHARED gazebo_ros_imu.cpp)
#   set_target_properties(gazebo_ros_imu PROPERTIES COMPILE_FLAGS "${sensor_msgs_CFLAGS_OTHER} ${std_srvs_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_imu PROPERTIES LINK_FLAGS "${sensor_msgs_LDFLAGS_OTHER} ${std_srvs_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_imu ${sensor_msgs_LIBRARIES} ${std_srvs_LIBRARIES} RayPlugin)
#   install (TARGETS gazebo_ros_imu DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   add_library(gazebo_ros_camera SHARED gazebo_ros_camera.cpp)
#   target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils)
#   install (TARGETS gazebo_ros_camera DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   include_directories(${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(MultiSenseSLPlugin SHARED MultiSenseSLPlugin.cpp)
#   set_target_properties(MultiSenseSLPlugin PROPERTIES COMPILE_FLAGS "${std_msgs_CFLAGS_OTHER} ${sensor_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(MultiSenseSLPlugin PROPERTIES LINK_FLAGS "${std_msgs_LDFLAGS_OTHER} ${sensor_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(MultiSenseSLPlugin gazebo_ros_camera_utils ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES} CameraPlugin)
#   install (TARGETS MultiSenseSLPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   get_rospack_flags(sensor_msgs)
#   get_rospack_flags(trajectory_msgs)
#   #get_rospack_flags(geometry_msgs)
#   #get_rospack_flags(image_transport)
#   #get_rospack_flags(tf)
#   include_directories(${nav_msgs_INCLUDE_DIRS} ${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${nav_msgs_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${nav_msgs_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(AtlasPlugin SHARED AtlasPlugin.cc)
#   set_target_properties(AtlasPlugin PROPERTIES COMPILE_FLAGS "${nav_msgs_CFLAGS_OTHER} ${trajectory_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(AtlasPlugin PROPERTIES LINK_FLAGS "${nav_msgs_LDFLAGS_OTHER} ${trajectory_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(AtlasPlugin ${nav_msgs_LIBRARIES} ${trajectory_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
#   install (TARGETS AtlasPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   get_rospack_flags(control_msgs)
#   get_rospack_flags(actionlib)
#   include_directories(${control_msgs_INCLUDE_DIRS} ${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${actionlib_LIBRARY_DIRS} ${control_msgs_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   add_library(VRCPlugin SHARED VRCPlugin.cc)
#   set_target_properties(VRCPlugin PROPERTIES COMPILE_FLAGS "${control_msgs_CFLAGS_OTHER} ${trajectory_msgs_CFLAGS_OTHER} ${sensor_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(VRCPlugin PROPERTIES LINK_FLAGS "${control_msgs_LDFLAGS_OTHER} ${trajectory_msgs_LDFLAGS_OTHER} ${sensor_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(VRCPlugin ${actionlib_LIBRARIES} ${control_msgs_LIBRARIES} ${trajectory_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
#   install (TARGETS VRCPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   add_library(DRCVehicleROSPlugin SHARED DRCVehicleROSPlugin.cc)
#   target_link_libraries(DRCVehicleROSPlugin DRCVehiclePlugin)
#   install (TARGETS DRCVehicleROSPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#  
#   #get_rospack_flags(trajectory_msgs)
#   #get_rospack_flags(geometry_msgs)
#   #get_rospack_flags(image_transport)
#   #get_rospack_flags(tf)
#   include_directories(${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(gazebo_ros_joint_trajectory SHARED gazebo_ros_joint_trajectory.cpp)
#   set_target_properties(gazebo_ros_joint_trajectory PROPERTIES COMPILE_FLAGS "${trajectory_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_joint_trajectory PROPERTIES LINK_FLAGS "${trajectory_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_joint_trajectory ${trajectory_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
#   install (TARGETS gazebo_ros_joint_trajectory DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#   #get_rospack_flags(trajectory_msgs)
#   #get_rospack_flags(geometry_msgs)
#   #get_rospack_flags(image_transport)
#   #get_rospack_flags(tf)
#   #get_rospack_flags(gazebo_msgs)
#   #include_directories(${gazebo_msgs_INCLUDE_DIRS} ${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   #link_directories(${gazebo_msgs_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   #set(CMAKE_INSTALL_RPATH "${gazebo_msgs_LIBRARY_DIRS} ${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   #add_library(gazebo_ros_api_plugin SHARED gazebo_ros_api_plugin.cpp)
#   #set_target_properties(gazebo_ros_api_plugin PROPERTIES COMPILE_FLAGS "${gazebo_msgs_CFLAGS_OTHER} ${trajectory_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   #set_target_properties(gazebo_ros_api_plugin PROPERTIES LINK_FLAGS "${gazebo_msgs_LDFLAGS_OTHER} ${trajectory_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   #target_link_libraries(gazebo_ros_api_plugin ${gazebo_msgs_LIBRARIES} ${trajectory_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
#   #install (TARGETS gazebo_ros_api_plugin DESTINATION ${gazebo_msgs_INCLUDE_DIRS} ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#   include_directories(${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
#   link_directories(${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
#   add_library(gazebo_ros_api_plugin SHARED gazebo_ros_api_plugin.cpp)
#   set_target_properties(gazebo_ros_api_plugin PROPERTIES COMPILE_FLAGS "${trajectory_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_api_plugin PROPERTIES LINK_FLAGS "${trajectory_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_api_plugin ${trajectory_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
#   install (TARGETS gazebo_ros_api_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
#  
#  
#   get_rospack_flags(nav_msgs)
#   include_directories(${trajectory_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS})
#   link_directories(${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS})
#   set(CMAKE_INSTALL_RPATH "${trajectory_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS}")
#   add_library(gazebo_ros_p3d SHARED gazebo_ros_p3d.cpp)
#   set_target_properties(gazebo_ros_p3d PROPERTIES COMPILE_FLAGS "${trajectory_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER} ${nav_msgs_CFLAGS_OTHER}")
#   set_target_properties(gazebo_ros_p3d PROPERTIES LINK_FLAGS "${trajectory_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER} ${nav_msgs_LDFLAGS_OTHER}")
#   target_link_libraries(gazebo_ros_p3d ${trajectory_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES} ${nav_msgs_LIBRARIES})
#   install (TARGETS gazebo_ros_p3d DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}-${DRC_SIM_VERSION}/plugins/)
