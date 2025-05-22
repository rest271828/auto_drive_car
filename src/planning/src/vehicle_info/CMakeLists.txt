cmake_minimum_required(VERSION 3.22)
project(vehicle_info)

#设置依赖---------------------------------------------------------------------------------
set(DEPENDS
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
  "base_msgs"
)

#添加动态链接库-------------------------------------------------------------------------------
add_library(vehicle_info SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/main_car/main_car_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/obs_car/obs_car_info.cpp
)

set(target
  vehicle_info
)
#添加依赖----------------------------------------------------------------------------------
target_link_libraries(${target}
  config_reader
  mathlibs
)

ament_target_dependencies(${target} 
${DEPENDS})
#安装动态库、节点和相关目录 --------------------------------------------------------------------
install(TARGETS
  LIBRARY DESTINATION lib
)