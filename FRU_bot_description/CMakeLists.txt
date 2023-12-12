cmake_minimum_required(VERSION 3.5)
project(FRU_bot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf urdf/meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
