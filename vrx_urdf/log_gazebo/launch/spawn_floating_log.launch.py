#!/usr/bin/env python3
# =============================================================================
# spawn_floating_log.launch.py
# =============================================================================
# Reads the coefficient YAML files in config/, processes log_gazebo.xacro with
# those values, and spawns the resulting model (with VRX buoyancy +
# hydrodynamics plugins baked in) into a running Gazebo / VRX simulation.
#
# Example:
#   ros2 launch log_gazebo spawn_floating_log.launch.py
#   ros2 launch log_gazebo spawn_floating_log.launch.py x:=10.0 y:=-5.0 z:=1.0
#
# The launch assumes Gazebo (VRX world) is already running. To bring up the
# bundled standalone test world instead, see worlds/examples/floating_log_test.sdf
# and the instructions in docs/VALIDATION.md.
# =============================================================================

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _flatten(d):
    """Flatten the single top-level YAML section into a {str: str} mapping."""
    # Each config file wraps its values under one key (buoyancy/hydrodynamics/wind).
    inner = next(iter(d.values())) if len(d) == 1 and isinstance(next(iter(d.values())), dict) else d
    return {str(k): str(v) for k, v in inner.items()}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('log_gazebo')
    cfg = os.path.join(pkg_share, 'config')
    xacro_file = os.path.join(pkg_share, 'urdf', 'log_gazebo.xacro')

    # --- Load coefficient configuration --------------------------------------
    mappings = {}
    for fname in ('buoyancy.yaml', 'hydrodynamics.yaml', 'wind.yaml'):
        path = os.path.join(cfg, fname)
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            mappings.update(_flatten(data))

    # Only pass mappings that the xacro actually declares as <xacro:arg>.
    accepted = {
        'hull_length', 'hull_radius', 'fluid_level', 'fluid_density',
        'wavefield_topic',
        'xDotU', 'yDotV', 'zDotW', 'kDotP', 'mDotQ', 'nDotR',
        'xU', 'yV', 'zW', 'kP', 'mQ', 'nR',
        'xUU', 'yVV', 'zWW', 'kPP', 'mQQ', 'nRR',
    }
    xacro_mappings = {k: v for k, v in mappings.items() if k in accepted}

    # Wind coefficient keys in wind.yaml use a different naming; the enable_wind
    # flag is the only wind value consumed inside the model xacro.
    xacro_mappings['enable_wind'] = LaunchConfiguration('enable_wind').perform(context)

    # --- Process xacro into a model description string -----------------------
    doc = xacro.process_file(xacro_file, mappings=xacro_mappings)
    robot_desc = doc.toxml()

    # --- Spawn into the running simulation -----------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_floating_log',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', LaunchConfiguration('name').perform(context),
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-z', LaunchConfiguration('z').perform(context),
            '-R', LaunchConfiguration('R').perform(context),
            '-P', LaunchConfiguration('P').perform(context),
            '-Y', LaunchConfiguration('Y').perform(context),
        ],
    )
    return [spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='floating_log',
                              description='Model name in the simulation.'),
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        # Spawn slightly above the surface so Test 2 (fall + float) is visible.
        DeclareLaunchArgument('z', default_value='10.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('enable_wind', default_value='true',
                              description='Opt the log in to the world wind system.'),
        OpaqueFunction(function=launch_setup),
    ])
