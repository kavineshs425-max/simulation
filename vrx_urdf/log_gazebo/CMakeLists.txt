cmake_minimum_required(VERSION 3.8)
project(log_gazebo)

find_package(ament_cmake REQUIRED)

# Install all asset directories.
install(
  DIRECTORY
    config
    launch
    urdf
    worlds
    docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
