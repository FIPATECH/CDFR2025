cmake_minimum_required(VERSION 3.5)
project(uart_bridge)

# Recherche des packages requis
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# Création de l'exécutable
add_executable(uart_bridge_node src/uart_bridge.cpp)
ament_target_dependencies(uart_bridge_node
  rclcpp
  std_msgs
  serial
)
target_link_libraries(uart_bridge_node serial)

# Installation de l'exécutable
install(TARGETS
  uart_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installation du dossier launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Installation du package.xml pour l'indexation ament
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
