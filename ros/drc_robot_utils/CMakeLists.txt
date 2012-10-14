set (ROS_PACKAGE_NAME drc_robot_utils)

set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/)
add_subdirectory(launch)
add_subdirectory(robots)
add_subdirectory(scripts)
add_subdirectory(urdf)
add_subdirectory(worlds)

# Build of bdiparser disabled for now, pending updated version of urdfdom library
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
INSTALL(TARGETS bdi_parser DESTINATION bin/)
