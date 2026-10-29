#!/usr/bin/env python3
# =============================================================================
# my_boat_world.launch.py
# =============================================================================
# One-shot launch that brings up the VRX environment and then spawns my_boat
# into it. Composes two existing launch files:
#   * vrx_gz/launch/vrx_environment.launch.py  -- the simulation + world
#   * my_boat_gazebo/launch/spawn_my_boat.launch.py -- the boat model
#
# The spawn is deferred a few seconds (TimerAction) so the Gazebo/VRX world is
# up and running before `ros_gz_sim create` tries to inject the model.
#
# Examples:
#   ros2 launch my_boat_gazebo my_boat_world.launch.py
#   ros2 launch my_boat_gazebo my_boat_world.launch.py world:=sydney_regatta headless:=True
#   ros2 launch my_boat_gazebo my_boat_world.launch.py x:=10.0 y:=-5.0 Y:=1.57
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vrx_gz_share = get_package_share_directory('vrx_gz')
    my_boat_share = get_package_share_directory('my_boat_gazebo')

    vrx_environment_launch = os.path.join(
        vrx_gz_share, 'launch', 'vrx_environment.launch.py')
    spawn_my_boat_launch = os.path.join(
        my_boat_share, 'launch', 'spawn_my_boat.launch.py')

    world = LaunchConfiguration('world')

    # --- Bring up the VRX environment (simulation + world) -------------------
    vrx_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vrx_environment_launch),
        launch_arguments={
            'world': world,
            'sim_mode': LaunchConfiguration('sim_mode'),
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
        }.items(),
    )

    # --- Spawn my_boat once the world is up ----------------------------------
    spawn_my_boat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_my_boat_launch),
        launch_arguments={
            'world': world,
            'name': LaunchConfiguration('name'),
            'namespace': LaunchConfiguration('namespace'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'R': LaunchConfiguration('R'),
            'P': LaunchConfiguration('P'),
            'Y': LaunchConfiguration('Y'),
            'bridge': LaunchConfiguration('bridge'),
        }.items(),
    )

    delayed_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[spawn_my_boat],
    )

    return LaunchDescription([
        # --- Environment args (forwarded to vrx_environment.launch.py) -------
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                              description='Name of world.'),
        DeclareLaunchArgument('sim_mode', default_value='full',
                              description='Simulation mode: "full", "sim", "bridge".'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True to run simulation headless (no GUI).'),
        DeclareLaunchArgument('paused', default_value='False',
                              description='True to start the simulation paused.'),

        # --- Boat spawn args (forwarded to spawn_my_boat.launch.py) ----------
        DeclareLaunchArgument('name', default_value='my_boat',
                              description='Model name in the sim (keep == namespace).'),
        DeclareLaunchArgument('namespace', default_value='my_boat',
                              description='Link/plugin namespace (link = <ns>/base_link).'),
        DeclareLaunchArgument('x', default_value='-528'),
        DeclareLaunchArgument('y', default_value='186'),
        DeclareLaunchArgument('z', default_value='1.5'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Start the thruster ROS<->GZ bridge.'),

        # --- Composition control ---------------------------------------------
        DeclareLaunchArgument('spawn_delay', default_value='5.0',
                              description='Seconds to wait for the world to come '
                                          'up before spawning the boat.'),

        vrx_environment,
        delayed_spawn,
    ])
