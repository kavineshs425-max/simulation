#!/usr/bin/env python3
# =============================================================================
# keyboard_teleop.launch.py
# =============================================================================
# Launches the my_boat keyboard teleop.
#
# IMPORTANT: keyboard teleop does NOT work through `ros2 launch`. launch
# redirects each node's stdin to a pipe, so the node never receives your
# keystrokes (the terminal stays in echo/cooked mode and just prints
# "wwww..."). emulate_tty only fakes the node's *output* TTY -- it does NOT
# provide an interactive stdin. Run the node directly in its own terminal:
#
#   ros2 run my_boat_teleop keyboard_teleop --ros-args \
#       -p control_mode:=actuator -p boat_namespace:=my_boat -p thrust_step:=1.0
#
# This launch file is kept only for parameter documentation / composition; for
# actual keyboard control use `ros2 run` as above.
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        'boat_namespace': 'my_boat',
        'control_mode': 'actuator',   # 'actuator' (per-actuator keys) or 'direction' (wasd intent)
        'thrust_step': '1.0',         # N per tap
        'steer_step_deg': '1.0',      # deg per tap
        'accel_gain': '0.6',          # step growth per auto-repeat while held
        'accel_max': '30.0',          # max step multiplier when held
        'hold_reset_ms': '150.0',     # idle gap that resets the hold acceleration
        # Bollard-measured, signed, ASYMMETRIC per thruster. T2's are the mirror
        # of T1's because it is physically mounted the other way round. Keep
        # these equal to allocator_bollard.yaml and my_boat_thruster_layout.xacro
        # -- Gazebo clamps to the xacro values, so a looser limit here only makes
        # the teleop status line overstate what the boat is actually getting.
        't1_min_thrust': '-39.2266',
        't1_max_thrust': '78.4532',
        't2_min_thrust': '-78.4532',
        't2_max_thrust': '39.2266',
        't3_min_thrust': '-274.6',
        't3_max_thrust': '554.1',
        'steer_max': '0.3665',
        'publish_rate': '20.0',
    }

    params = {k: LaunchConfiguration(k) for k in args}

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
        + [
            Node(
                package='my_boat_teleop',
                executable='keyboard_teleop',
                name='keyboard_teleop',
                output='screen',
                emulate_tty=True,      # keep the TTY attached for raw keyboard input
                parameters=[params],
            ),
        ]
    )
