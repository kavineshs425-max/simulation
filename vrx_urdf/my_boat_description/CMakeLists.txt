cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(my_boat_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Environment hooks so Gazebo can resolve model:// and package:// URIs that
# point into this package's models directory (mirrors wamv_description).
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Generate a plain URDF from the xacro at build time and install it.
xacro_add_files(
  urdf/my_boat_base.urdf.xacro
    INSTALL DESTINATION urdf
)

# Install meshes, models, urdf sources, config and docs.
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs)

ament_package()
