import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare the launch arguments
    arg_cmd_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # Launch RViz
    node_cmd_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', os.path.join(get_package_share_directory('aictrain_base'), 'rviz', 'view_nav.rviz')])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(arg_cmd_use_sim_time)
    ld.add_action(node_cmd_rviz)

    return ld
