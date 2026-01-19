cmake_minimum_required(VERSION 3.28.3)
project(decision_center)

# 设置依赖
set(DEPENDS
  "rclcpp"
  "ament_index_cpp" 
)

# 添加动态库
add_library(decision_center SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_center.cpp
)

set(target
  decision_center
)

# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  vehicle_info
)

# 安装动态库，节点，相关目录
install(TARGETS
  ${target}
  LIBRARY DESTINATION lib
)

