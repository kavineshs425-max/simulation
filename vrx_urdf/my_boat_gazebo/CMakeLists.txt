cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(my_boat_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(my_boat_description REQUIRED)

# Install asset directories (xacro macros, configs, worlds, launch, docs).
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs)

# Static obstacle models, spawned at runtime by launch/spawn_obstacles.launch.py.
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
