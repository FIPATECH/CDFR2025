cmake_minimum_required(VERSION 3.10)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY launch urdf meshes config
        DESTINATION share/${PROJECT_NAME})

ament_package()