cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo)

find_package(catkin REQUIRED COMPONENTS atlas_description drcsim_gazebo_plugins atlas_msgs drcsim_gazebo_ros_plugins drcsim_model_resources irobot_hand_description multisense_sl_description sandia_hand_description)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS atlas_description drcsim_gazebo_plugins atlas_msgs drcsim_gazebo_ros_plugins drcsim_model_resources irobot_hand_description multisense_sl_description sandia_hand_description
)

install(DIRECTORY
  launch config raw
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install test files separately since it contains some executable scripts
install(FILES
  test/CheckDisplayCapabilities.cmake
  test/vrc_task_2_dynamic_walking.test
  test/atlas_cheats_rosapi.yaml
  test/vrc_task_2_gzlog_stop.test
  test/performance_test1.test
  test/vrc_task_1_rosapi.test
  test/atlas_drc_vehicle_fire_hose_rosapi.test
  test/atlas_sandia_hands_rosapi.test
  test/vrc_task_1_dynamic_walking.test
  test/pub_atlas_command.launch
  test/multisense_sl_rosapi.yaml
  test/CMakeLists.txt
  test/drc_vehicle_cheats_rosapi.yaml
  test/atlas_arenas_start_testing.launch
  test/multicamera_subscriber
  test/vrc_task_2_scoring.test
  test/vrc_rosapi.yaml
  test/atlas_arenas_logging.launch
  test/atlas_rosapi.yaml
  test/atlas_cheats_rosapi.test
  test/vrc_task_2_tricking_scoring.test
  test/perf_test_local.launch
  test/vrc_task_1_gzlog_stop_longrun_zlib.test
  test/vrc_task_3_rosapi.test
  test/vrc_task_1_gzlog_stop_longrun.test
  test/vrc_task_3_dynamic_walking.test
  test/atlas_golf_cart_fire_hose_rosapi.test
  test/atlas_sandia_hands_cheats_rosapi.test
  test/vrc_task_1_start_standup.test
  test/vrc_task_1_scoring.test
  test/vrc_task_3_cheats_rosapi.test
  test/vrc_task_1_gzlog_stop.test
  test/multicamera_connection.test
  test/atlas_rosapi.test
  test/atlas_sandia_hands_publishers_hz.test
  test/sandia_hands_rosapi.yaml
  test/vrc_task_1_cheats_rosapi.test
  test/vrc_task_3_start_standup.test
  test/atlas_sandia_hands_publishers_hz_gpu.test
  test/vrc_task_2_rosapi.test
  test/vrc_task_1_zlib_compression.launch
  test/vrc_task_2_start_standup.test
  test/atlas_publishers_hz.test
  test/atlas_publishers_hz_gpu.test
  test/golf_cart_cheats_rosapi.yaml
  test/vrc_task_2_cheats_rosapi.test
  test/vrc_task_1_commander.launch
  test/ros_subscribers.launch
  test/vrc_task_3_gzlog_stop.test
  test/perf_test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

# install scripts and test executable scripts
install(PROGRAMS
  test/gl-test.py
  test/gzlog_stop_checker.py
  test/meanvar.py
  test/run_pub_atlas_command.sh
  test/run_gzstats.sh
  test/run_rosbag.sh
  test/run_recordmydesktop.sh
  test/parse_logs.sh
  test/vrc_task_2_scoring_test
  test/vrc_task_2_tricking_scoring_test
  test/vrc_walking_test
  test/vrc_task_1_scoring_test
  test/ros_api_checker
  test/scoring_checker
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

install(PROGRAMS
  scripts/orientation_checker.py
  scripts/test.py
  scripts/keyboard_teleop.py
  scripts/run_gzserver_gdb
  scripts/CMakeLists.txt
  scripts/gdbrun
  scripts/nanokontrol.py
  scripts/run_gzserver_cheats
  scripts/run_gazebo
  scripts/run_gzclient
  scripts/atlas_commander.py
  scripts/run_gazebo_paused
  scripts/performance_test1.py
  scripts/run_gzserver_perf
  scripts/arm_teleop.py
  scripts/run_gzserver_callgrind
  scripts/reset_pose.py
  scripts/leg_teleop.py
  scripts/stand.py
  scripts/run_gzserver
  scripts/5steps.py
  scripts/run_gzserver_memcheck
  scripts/check_inertia_symmetry.bash
  scripts/spawn_model
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

# disabled build of bdiparser for now, pending updated version of urdfdom library?
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SearchForTinyXML)
include_directories(${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

find_package(urdfdom REQUIRED)
include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
link_directories(${urdfdom_LIBRARY_DIRS})

find_package(urdfdom_headers REQUIRED)
include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})

find_package(console_bridge REQUIRED)
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
link_directories(${console_bridge_LIBRARY_DIRS})

find_package(Boost REQUIRED thread)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(bdi_parser src/bdi_parser.cpp)
target_link_libraries(bdi_parser ${urdfdom_LIBRARIES} ${tinyxml_libraries} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

# INSTALL(TARGETS bdi_parser DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/bin/)
install(TARGETS bdi_parser DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/)

