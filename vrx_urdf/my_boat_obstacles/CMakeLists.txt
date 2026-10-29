cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(my_boat_obstacles)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)

# Gazebo system plugin: plays a timed trajectory back on a model.
add_library(MovingObstacle SHARED src/MovingObstacle.cc)
target_compile_features(MovingObstacle PUBLIC cxx_std_17)
target_link_libraries(MovingObstacle PUBLIC
  gz-sim7::core
  gz-plugin2::register
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs9::gz-msgs9
  gz-transport12::gz-transport12)
install(TARGETS MovingObstacle DESTINATION lib)

# so <plugin filename="MovingObstacle"> also resolves by name
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/plugin_path.dsv.in")

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/obstacle_spawner scripts/obstacle_ui scripts/sim_environment DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch scenarios config maps models worlds ui DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/make_land_mask.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_core test/test_core.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
