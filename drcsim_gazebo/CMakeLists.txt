cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo)
set(CMAKE_BUILD_TYPE "Debug")


# Used by test/ and script/
set(ROS_PACKAGE_NAME drcsim_gazebo)

find_package(catkin REQUIRED COMPONENTS atlas_description drcsim_gazebo_plugins atlas_msgs drcsim_gazebo_ros_plugins drcsim_model_resources irobot_hand_description multisense_sl_description sandia_hand_description)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS atlas_description drcsim_gazebo_plugins atlas_msgs drcsim_gazebo_ros_plugins drcsim_model_resources irobot_hand_description multisense_sl_description sandia_hand_description
)

install(DIRECTORY
  launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

# disabled build of bdiparser for now, pending updated version of urdfdom library?
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SearchForTinyXML)
include_directories(${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

# needed by bdi_parser
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

install(TARGETS bdi_parser DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/)

# Include test subdirectory
add_subdirectory(test)
