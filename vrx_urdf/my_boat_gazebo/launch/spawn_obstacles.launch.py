#!/usr/bin/env python3
"""
spawn_obstacles.launch.py -- drop the static obstacle field into a RUNNING sim.

    # terminal 1
    ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
    # terminal 2, once the boat has spawned
    ros2 launch my_boat_gazebo spawn_obstacles.launch.py

WHY THIS IS A SEPARATE LAUNCH, AND WHY OBSTACLES VANISH ON RESTART

`ros_gz_sim create` injects a model into the world that is running right now.
Nothing is written back to the world file, so every sim restart begins with a
clean world and the obstacles are gone. That is not a bug to fix -- it is what
keeps sydney_regatta.sdf untouched. Re-run this launch after each sim start,
or add it to your own bringup.

The obstacles are <static>true</static>: on water a body with no buoyancy
plugin sinks, and static also means the GUI Translate tool leaves them exactly
where you drop them. Move them by hand for a one-off, or edit
config/obstacles.yaml to change where they start.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('my_boat_gazebo')
    cfg = LaunchConfiguration('config_file').perform(context)
    if not cfg:
        cfg = os.path.join(share, 'config', 'obstacles.yaml')
    world = LaunchConfiguration('world').perform(context)

    with open(cfg) as fh:
        items = (yaml.safe_load(fh) or {}).get('obstacles', []) or []

    actions = []
    for i, o in enumerate(items):
        sdf = os.path.join(share, 'models', 'obstacles',
                           'obstacle_%s.sdf' % o['type'])
        if not os.path.exists(sdf):
            raise RuntimeError(
                "unknown obstacle type '%s' for '%s' -- no %s"
                % (o['type'], o.get('name', '?'), sdf))
        yaw = float(o.get('yaw', 0.0)) * 3.141592653589793 / 180.0
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            name='spawn_obstacle_%d' % i,
            arguments=['-world', world, '-file', sdf,
                       '-name', str(o['name']),
                       '-x', str(o['x']), '-y', str(o['y']), '-z', '0',
                       '-Y', str(yaw)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta'),
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='Override config/obstacles.yaml with your own field.'),
        OpaqueFunction(function=_setup),
    ])
