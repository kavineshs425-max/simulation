r"""Drop a scripted, MOVING obstacle scenario into the running my_boat sim.

    # terminal 1
    ros2 launch my_boat_gazebo my_boat_world_v2.launch.py
    # terminal 2: obstacles from config/obstacles.yaml appear and HOLD STILL
    ros2 launch my_boat_obstacles moving_obstacles.launch.py
    #   (or config:=crossing_starboard, or config:=/path/to/my_file.yaml)
    # start them moving (again = restart from the start poses)
    ros2 service call /obstacle_spawner/run std_srvs/srv/Trigger
    # back to the start poses, holding
    ros2 service call /obstacle_spawner/reset std_srvs/srv/Trigger
    # move one obstacle's start pose (yaw 90 deg = north: z=0.7071, w=0.7071)
    ros2 service call /obstacle_spawner/set_start ros_gz_interfaces/srv/SetEntityPose \
      "{entity: {name: crosser}, pose: {position: {x: -470.0, y: 150.0}, orientation: {z: 0.7071, w: 0.7071}}}"

    # the SIMU browser UI starts too: open http://localhost:8090  (ui:=false to skip)
    # after editing the config file: apply it without restarting
    ros2 service call /obstacle_spawner/reload std_srvs/srv/Trigger

Start poses are absolute Gazebo world coordinates.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('my_boat_obstacles')
    cfg = LaunchConfiguration('config').perform(context)
    if not os.path.isfile(cfg):
        name = cfg if cfg.endswith('.yaml') else cfg + '.yaml'
        for folder in ('config', 'scenarios'):
            if os.path.isfile(os.path.join(share, folder, name)):
                cfg = os.path.join(share, folder, name)
                break
    if not os.path.isfile(cfg):
        raise RuntimeError(f'moving_obstacles: config not found: {cfg}')
    from my_boat_obstacles.catalog import RUN_TOPIC, SAMPLES_TOPIC
    env = LaunchConfiguration('environment').perform(context)
    env_nodes = []
    if env.lower() not in ('none', 'false', 'off', ''):
        if not os.path.isfile(env):
            names = [env] if env.endswith(('.yaml', '.geojson')) else [env + '.geojson', env + '.yaml']
            found = [os.path.join(share, d, n) for d in ('maps', 'config') for n in names
                     if os.path.isfile(os.path.join(share, d, n))]
            env = found[0] if found else env
        if not os.path.isfile(env):
            raise RuntimeError(f'moving_obstacles: environment not found: {env}')
        # the fixed world: canals, basins, land, docks, bridges -- from the VECTOR MAP
        # (maps/regatta_canals.geojson), whose information features ride along
        env_nodes.append(Node(package='my_boat_obstacles', executable='sim_environment',
                              name='sim_environment', output='screen',
                              parameters=[{'use_sim_time': True, 'config_file': env}]))
    return env_nodes + [
        # spawner (ROS) -> every MovingObstacle plugin (gz): the run signal,
        # and replacement paths from ~/set_start and ~/reload
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='obstacle_run_bridge', output='screen',
             arguments=[f'{RUN_TOPIC}@std_msgs/msg/Float64]gz.msgs.Double',
                        f'{SAMPLES_TOPIC}@std_msgs/msg/String]gz.msgs.StringMsg']),
        Node(package='my_boat_obstacles', executable='obstacle_spawner',
             name='obstacle_spawner', output='screen',
             parameters=[{'use_sim_time': True, 'config_file': cfg}]),
    ] + ([
        # SIMU browser UI: http://localhost:<ui_port>
        Node(package='my_boat_obstacles', executable='obstacle_ui', name='obstacle_ui',
             output='screen',
             parameters=[{'port': int(LaunchConfiguration('ui_port').perform(context)),
                          'host': LaunchConfiguration('ui_host').perform(context)}]),
    ] if LaunchConfiguration('ui').perform(context).lower() in ('true', '1') else [])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='obstacles',
            description='config file: a path, or a name from this package\'s config/ '
                        'or scenarios/ folder. Default: config/obstacles.yaml'),
        DeclareLaunchArgument(
            'environment', default_value='regatta_canals',
            description='the fixed environment: a vector map (.geojson) or environment .yaml -- a '
                        'path, or a name from maps/ or config/. Default maps/regatta_canals.geojson; '
                        'none = no environment'),
        DeclareLaunchArgument('ui', default_value='true',
                              description='start the SIMU browser UI (obstacle_ui)'),
        DeclareLaunchArgument('ui_port', default_value='8090'),
        DeclareLaunchArgument('ui_host', default_value='127.0.0.1',
                              description='0.0.0.0 to open the UI from another machine'),
        OpaqueFunction(function=_setup),
    ])
