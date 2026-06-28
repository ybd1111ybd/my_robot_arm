cmake_minimum_required(VERSION 3.10...3.30)
project(jz_robot_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch meshes scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
