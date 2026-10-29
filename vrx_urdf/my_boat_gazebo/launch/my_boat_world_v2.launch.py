#!/usr/bin/env python3
# =============================================================================
# my_boat_world_v2.launch.py
# =============================================================================
# One-shot launch that brings up the VRX environment and then spawns my_boat
# with the **v2 decoupled thruster arrangement**:
#
#     T1  forward lateral thruster  (centreline, d1 = 1.40 m FWD of the CG)
#     T2  aft     lateral thruster  (centreline, d2 = 0.90 m AFT of the CG)
#     P1  fixed axial stern propeller (d3 = 1.40 m aft, lateral offset p1_y)
#
# The stations default to thruster_allocator_v2/config/allocator_v2.yaml. Keep
# the two in sync -- the allocator's B and the simulated hull must describe the
# same boat, or every yaw command is biased by the ratio of the two d's.
#
# v2 counterpart of my_boat_world.launch.py (which keeps the v1 layout: two bow
# tunnels side by side + a steerable stern drive). Composes two launch files:
#   * vrx_gz/launch/vrx_environment.launch.py         -- the simulation + world
#   * my_boat_gazebo/launch/spawn_my_boat_v2.launch.py -- the v2 boat model
#
# The spawn is deferred a few seconds (TimerAction) so the Gazebo/VRX world is
# up and running before `ros_gz_sim create` tries to inject the model.
#
# Examples:
#   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
#   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py headless:=True
#   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py x:=10.0 y:=-5.0 Y:=1.57
#   # retune the thruster stations (keep allocator_v2.yaml in sync!):
#   ros2 launch my_boat_gazebo my_boat_world_v2.launch.py d1:=1.9 d2:=1.4
#
# Drive it (thrust in Newtons; see docs/THRUSTERS_V2.md):
#   ros2 topic pub -1 /my_boat/thrusters/P1/thrust std_msgs/msg/Float64 "{data: 400.0}"   # ahead
#   ros2 topic pub -1 /my_boat/thrusters/T1/thrust std_msgs/msg/Float64 "{data: 98.0}"    # +yaw
#   ros2 topic pub -1 /my_boat/thrusters/T2/thrust std_msgs/msg/Float64 "{data: -98.0}"   # +yaw
#
# Or drive it from the DP stack / allocator instead of by hand:
#   ros2 launch dp_control_system_v2 system_bringup_v2.launch.py
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
        my_boat_share, 'launch', 'spawn_my_boat_v2.launch.py')

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

    # --- Spawn the v2 boat once the world is up ------------------------------
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
            'd1': LaunchConfiguration('d1'),
            'd2': LaunchConfiguration('d2'),
            'd3': LaunchConfiguration('d3'),
            'p1_y': LaunchConfiguration('p1_y'),
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

        # --- Boat spawn args (forwarded to spawn_my_boat_v2.launch.py) -------
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
                              description='Start the thruster/sensor ROS<->GZ bridges.'),

        # --- v2 thruster stations (metres, base_link/CG frame) ---------------
        # Defaults mirror thruster_allocator_v2/config/allocator_v2.yaml.
        DeclareLaunchArgument('d1', default_value='1.40',
                              description='T1 forward lateral thruster, distance '
                                          'FORWARD of the CG [m].'),
        DeclareLaunchArgument('d2', default_value='0.90',
                              description='T2 aft lateral thruster, distance AFT '
                                          'of the CG [m].'),
        DeclareLaunchArgument('d3', default_value='1.40',
                              description='P1 stern propeller, distance AFT of '
                                          'the CG [m] (does not enter B).'),
        DeclareLaunchArgument('p1_y', default_value='0.00',
                              description='P1 lateral offset [m], + = to port.'),

        # --- Composition control ---------------------------------------------
        DeclareLaunchArgument('spawn_delay', default_value='5.0',
                              description='Seconds to wait for the world to come '
                                          'up before spawning the boat.'),

        vrx_environment,
        delayed_spawn,
    ])
