cmake_minimum_required(VERSION 3.8)
project(log_description)

find_package(ament_cmake REQUIRED)

# Install hook files for environment variable setup
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.sh"
)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in"
)

# Install package directories
install(
  DIRECTORY
    urdf
    meshes
    models
    hooks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
