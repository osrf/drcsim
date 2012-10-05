include(cmake/rospack.cmake)

get_rospack_flags(pr2_controller_manager)
include_directories(${pr2_controller_manager_INCLUDE_DIRS})
link_directories(${pr2_controller_manager_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${pr2_controller_manager_LIBRARY_DIRS}")
add_library(gazebo_ros_controller_manager SHARED gazebo_ros_controller_manager.cpp)
set_target_properties(gazebo_ros_controller_manager PROPERTIES COMPILE_FLAGS "${pr2_controller_manager_CFLAGS_OTHER}")
set_target_properties(gazebo_ros_controller_manager PROPERTIES LINK_FLAGS "${pr2_controller_manager_LDFLAGS_OTHER}")
target_link_libraries(gazebo_ros_controller_manager ${pr2_controller_manager_LIBRARIES})
install (TARGETS gazebo_ros_controller_manager DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/plugins/)


get_rospack_flags(std_msgs)
get_rospack_flags(sensor_msgs)
get_rospack_flags(geometry_msgs)
get_rospack_flags(image_transport)
get_rospack_flags(tf)
include_directories(${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
link_directories(${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${std_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS} ${image_transport_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
add_library(gazebo_ros_camera_utils SHARED gazebo_ros_camera_utils.cpp)
set_target_properties(gazebo_ros_camera_utils PROPERTIES COMPILE_FLAGS "${std_msgs_CFLAGS_OTHER} ${sensor_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
set_target_properties(gazebo_ros_camera_utils PROPERTIES LINK_FLAGS "${std_msgs_LDFLAGS_OTHER} ${sensor_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
target_link_libraries(gazebo_ros_camera_utils ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${tf_LIBRARIES})
install (TARGETS gazebo_ros_camera_utils DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/plugins/)


add_library(gazebo_ros_camera SHARED gazebo_ros_camera.cpp)
#set_target_properties(gazebo_ros_camera PROPERTIES COMPILE_FLAGS "${std_msgs_CFLAGS_OTHER} ${sensor_msgs_CFLAGS_OTHER} ${geometry_msgs_CFLAGS_OTHER} ${image_transport_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
#set_target_properties(gazebo_ros_camera PROPERTIES LINK_FLAGS "${std_msgs_LDFLAGS_OTHER} ${sensor_msgs_LDFLAGS_OTHER} ${geometry_msgs_LDFLAGS_OTHER} ${image_transport_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils)
install (TARGETS gazebo_ros_camera DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/plugins/)


#set (ros_plugins
#  test_ros_plugin
#)
#
#foreach (src ${ros_plugins})
#  add_library(${src} SHARED ${src}.cc)
#  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
#  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/drc_sim-${DRC_SIM_VERSION}/ros_plugins/)
#endforeach (src ${ros_plugins})
#
