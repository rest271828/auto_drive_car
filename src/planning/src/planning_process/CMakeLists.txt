cmake_minimum_required(VERSION 3.22)
project(planning_process)

#设置依赖---------------------------------------------------------------------------------
set(DEPENDS
  "rclcpp"
  "base_msgs"
  "nav_msgs"
  "tf2_ros"
  "tf2"
)

#添加节点-------------------------------------------------------------------------------
add_executable(planning_process
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp

)

set(target
  planning_process
)

#添加依赖----------------------------------------------------------------------------------
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  vehicle_info
  decision_center
  local_planner
  reference_line
)

#安装动态库、节点和相关目录 --------------------------------------------------------------------
install(TARGETS
  ${target}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)