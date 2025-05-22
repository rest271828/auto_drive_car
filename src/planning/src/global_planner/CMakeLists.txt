cmake_minimum_required(VERSION 3.22)
project(global_planner)

#设置依赖---------------------------------------------------------------------------------
set(DEPENDS
  "rclcpp"
  "base_msgs"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
)

#添加动态链接库-------------------------------------------------------------------------------
add_library(global_planner SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/global_planner_normal/global_planner_normal.cpp
)
#添加节点----------------------------------------------------------------------------------
add_executable(global_path_server
  ${CMAKE_CURRENT_SOURCE_DIR}/global_path_server.cpp
)

set(project_name
  global_planner
  global_path_server
)
#添加依赖---------------------------------------------------------------------------------
foreach(target ${project_name})
  ament_target_dependencies(${target} ${DEPENDS})
endforeach()

target_link_libraries(global_path_server
  global_planner
  config_reader
  )
#安装动态库、节点和相关目录 --------------------------------------------------------------------
install(TARGETS 
  global_planner
  LIBRARY DESTINATION lib
)

install(TARGETS 
  global_path_server
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)