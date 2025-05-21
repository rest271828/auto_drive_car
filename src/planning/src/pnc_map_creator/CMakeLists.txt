cmake_minimum_required(VERSION 3.22)
project(pnc_map_creator)

#设置依赖---------------------------------------------------------------------------------
set(DEPENDS
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
  "base_msgs"
)

#添加动态链接库-------------------------------------------------------------------------------
add_library(
  pnc_map_creator SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_straight/pnc_map_creator_straight.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_sturn/pnc_map_creator_sturn.cpp
)

set(
  LIB_TARGETS
  pnc_map_creator  
  )
#添加节点-------------------------------------------------------------------------------
add_executable(pnc_map_server
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_server.cpp
)

set(
  NODE_TARGETS
  pnc_map_server
)

set(PROJECT_TARGETS
  pnc_map_creator
  pnc_map_server
)
#添加依赖----------------------------------------------------------------------------------
foreach(target ${LIB_TARGETS})
  ament_target_dependencies(${target} ${DEPENDS})
  
endforeach()

target_link_libraries(pnc_map_server
  config_reader
  pnc_map_creator
)

#安装动态库、节点和相关目录 --------------------------------------------------------------------
install(TARGETS
  pnc_map_creator
  LIBRARY DESTINATION lib
)

install(
  TARGETS
  pnc_map_server
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)