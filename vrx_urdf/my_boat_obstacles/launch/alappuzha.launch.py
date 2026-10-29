"""Alappuzha (Kerala) backwater traffic: the environment + the traffic, on the running Sydney Regatta sim.

  ros2 launch my_boat_obstacles alappuzha.launch.py [ui:=true] [planners:=false]

  environment  maps/alappuzha.geojson  -- the Vadai / Commercial canal through the town (18 m, 5 kn, two
               bridges with 3 m clearance, KSWTD boat jetty) opening into Punnamada / Vembanad lake
               (stake nets); the SAME file is route 1's chart
  traffic      scenarios/alappuzha.yaml -- 18 craft (ferries, houseboats, shikaras, a cargo boat, a motor
               country boat, a speed boat, canoes, kayaks, drifting hyacinth) + 12 moored houseboats
  planners:=true  also starts route planner 1 + 2 (route_two.launch.py map:=alappuzha); otherwise run it
               yourself with map:=alappuzha so route 1 plans on this chart, not regatta_canals

Both files are GENERATED from scenarios/alappuzha/plan.py (python3 plan.py build; python3 plan.py draws
the plan). Our boat starts at its usual spawn (-528, 186) facing east: the canal mouth. Mission: send a
single goal to the free terminal berth (272, 280) in the SIMU page, then Run the obstacles.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    me = get_package_share_directory('my_boat_obstacles')
    out = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'moving_obstacles.launch.py')),
        launch_arguments={'config': os.path.join(me, 'scenarios', 'alappuzha.yaml'),
                          'environment': os.path.join(me, 'maps', 'alappuzha.geojson'),
                          'ui': LaunchConfiguration('ui').perform(context)}.items())]
    if LaunchConfiguration('planners').perform(context).lower() in ('true', '1'):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('route_two_planner'), 'launch', 'route_two.launch.py')),
            launch_arguments={'map': os.path.join(me, 'maps', 'alappuzha.geojson')}.items()))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ui', default_value='true', description='start the SIMU browser UI'),
        DeclareLaunchArgument('planners', default_value='false',
                              description='also start route planner 1 + 2 on the Alappuzha chart'),
        OpaqueFunction(function=_setup),
    ])
