cmake_minimum_required(VERSION 3.5)
project(match_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(match_control_node src/match_control.cpp)
ament_target_dependencies(match_control_node
  rclcpp
  std_msgs
  yaml-cpp
  ament_index_cpp
)
target_link_libraries(match_control_node yaml-cpp)

install(TARGETS
  match_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
