import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    pkg_base_project = get_package_share_directory('aictrain_base')

    map_yaml_file = LaunchConfiguration('map')

    # Declare the launch arguments
    arg_cmd_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_base_project, 'maps', 'rectangle', 'map.yaml'),
        description='Full path to map yaml file to load')

    group_cmd_bringup = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_base_project, 'launch', 'fake_robot.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_base_project, 'launch', 'minimal_nav.launch.py')),
            launch_arguments={'map': map_yaml_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_base_project, 'launch', 'view_nav.launch.py'))),
    ])

    # Create the launch description and populate
    return LaunchDescription([
        arg_cmd_map_yaml,
        group_cmd_bringup
    ])
