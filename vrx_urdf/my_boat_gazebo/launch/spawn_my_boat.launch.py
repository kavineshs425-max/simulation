#!/usr/bin/env python3
# =============================================================================
# spawn_my_boat.launch.py
# =============================================================================
# Processes my_boat_gazebo.urdf.xacro with the coefficients from config/*.yaml
# (buoyancy + hydrodynamics) and spawns the resulting model -- with the VRX
# Surface + SimpleHydrodynamics plugins baked in -- into an ALREADY RUNNING
# Gazebo / VRX simulation. Mirrors the VRX/log spawn pattern.
#
# Examples:
#   # Start a VRX world first, e.g.:
#   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
#   # then spawn the boat:
#   ros2 launch my_boat_gazebo spawn_my_boat.launch.py
#   ros2 launch my_boat_gazebo spawn_my_boat.launch.py x:=10.0 y:=-5.0 z:=1.5 Y:=1.57
#
# Keep -name equal to the namespace (default "my_boat") so the link
# "my_boat/base_link" matches the world's USVWind <wind_obj> link_name.
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
    """Flatten a single-section config dict ({buoyancy: {...}}) to {str: str}."""
    inner = next(iter(d.values())) if (len(d) == 1 and isinstance(next(iter(d.values())), dict)) else d
    return {str(k): str(v) for k, v in inner.items()}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_boat_gazebo')
    cfg = os.path.join(pkg_share, 'config')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_boat_gazebo.urdf.xacro')

    # --- Load coefficient configuration --------------------------------------
    mappings = {}
    for fname in ('buoyancy.yaml', 'hydrodynamics.yaml'):
        path = os.path.join(cfg, fname)
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            mappings.update(_flatten(data))

    # Only forward keys that the xacro actually declares as <xacro:arg>.
    accepted = {
        # PolyhedraBuoyancyDrag
        'fluid_level', 'fluid_density', 'linear_drag', 'angular_drag',
        'wavefield_topic',
        'buoy_size_x', 'buoy_size_y', 'buoy_size_z', 'buoy_pose',
        # SimpleHydrodynamics
        'xDotU', 'yDotV', 'zDotW', 'kDotP', 'mDotQ', 'nDotR',
        'xU', 'yV', 'zW', 'kP', 'mQ', 'nR',
        'xUU', 'yVV', 'zWW', 'kPP', 'mQQ', 'nRR',
    }
    xacro_mappings = {k: v for k, v in mappings.items() if k in accepted}
    xacro_mappings['namespace'] = LaunchConfiguration('namespace').perform(context)

    # --- Process xacro into a model description string -----------------------
    doc = xacro.process_file(xacro_file, mappings=xacro_mappings)
    robot_desc = doc.toxml()

    # --- Spawn into the running simulation -----------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_my_boat',
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

    nodes = [spawn]

    name = LaunchConfiguration('name').perform(context)
    namespace = xacro_mappings['namespace']
    world = LaunchConfiguration('world').perform(context)

    # --- robot_state_publisher (TF tree for the boat) ------------------------
    # Publishes the kinematic TF tree from the URDF: base_link -> the three
    # propeller links and the steerable T3 leg. It reads the live joint angles
    # from /<ns>/joint_states (fed by the gz JointStatePublisher via the bridge
    # below) so the propeller spin + steering show up in TF / RViz. The URDF
    # link names already carry the "<ns>/" prefix, so no frame_prefix is set
    # (that would double it to "<ns>/<ns>/base_link"). Parity with VRX/WAM-V.
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        remappings=[('/joint_states', f'/{namespace}/joint_states')],
    ))

    # --- ROS<->GZ bridges (thruster commands + IMU/GPS sensors) --------------
    # Bridges /my_boat/thrusters/{T1,T2,T3}/thrust + /T3/pos (commands) and the
    # IMU (/imu/data) and GPS (/gps/fix) sensor streams. Toggle bridge:=false.
    if LaunchConfiguration('bridge').perform(context).lower() in ('true', '1'):
        # joint_states: gz JointStatePublisher (in the URDF) -> ROS, so that
        # robot_state_publisher above sees the live joint angles. The gz topic
        # is world-scoped, hence the 'world' launch arg.
        gz_joint_topic = f'/world/{world}/model/{name}/joint_state'
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='my_boat_joint_state_bridge',
            output='screen',
            arguments=[
                f'{gz_joint_topic}@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[(gz_joint_topic, f'/{namespace}/joint_states')],
        ))
        thruster_cfg = os.path.join(cfg, 'thrusters.yaml')
        if os.path.exists(thruster_cfg):
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='my_boat_thruster_bridge',
                output='screen',
                parameters=[{'config_file': thruster_cfg}],
            ))
        sensors_cfg = os.path.join(cfg, 'sensors.yaml')
        if os.path.exists(sensors_cfg):
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='my_boat_sensor_bridge',
                output='screen',
                parameters=[{'config_file': sensors_cfg}],
            ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='my_boat',
                              description='Model name in the sim (keep == namespace).'),
        DeclareLaunchArgument('namespace', default_value='my_boat',
                              description='Link/plugin namespace (link = <ns>/base_link).'),
        DeclareLaunchArgument('x', default_value='-528'),
        DeclareLaunchArgument('y', default_value='186'),
        # Spawn above the surface so the fall-and-float test (Test 2) is visible.
        DeclareLaunchArgument('z', default_value='1.5'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Start the thruster ROS<->GZ bridge.'),
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                              description='Running world name; needed for the '
                                          'world-scoped joint_state gz topic.'),
        OpaqueFunction(function=launch_setup),
    ])
