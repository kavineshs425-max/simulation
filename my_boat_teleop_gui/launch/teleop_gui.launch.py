"""ros2 launch my_boat_teleop_gui teleop_gui.launch.py [control_mode:=direction] [thrust_step:=20.0]

Unlike the terminal teleop, this one works under `ros2 launch`: it reads the
keyboard from X11, not from stdin.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [('control_mode', 'direction'), ('thrust_step', '20.0'), ('boat_namespace', 'my_boat'),
            ('return_to_zero', 'true')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [
        Node(package='my_boat_teleop_gui', executable='teleop_gui', name='keyboard_teleop_gui',
             output='screen',
             parameters=[{'control_mode': LaunchConfiguration('control_mode'),
                          'thrust_step': LaunchConfiguration('thrust_step'),
                          'boat_namespace': LaunchConfiguration('boat_namespace'),
                          'return_to_zero': LaunchConfiguration('return_to_zero')}]),
    ])
