import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


AIC_MODEL = os.environ['AIC_MODEL']


def generate_launch_description():
    # Get the launch directory
    pkg_this_project = get_package_share_directory('aictrain_base')
    pkg_aicmodel = get_package_share_directory(AIC_MODEL)

    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    arg_cmd_urdf_model = DeclareLaunchArgument(
        name='urdf_model', default_value=os.path.join(pkg_aicmodel, 'urdf', 'index.xacro'),
        description='Absolute path to robot urdf file')

    arg_cmd_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_this_project, 'params', 'basic_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    node_cmd_fake_robot = Node(
        package='turtlebot3_fake_node',
        executable='turtlebot3_fake_node',
        parameters=[params_file],
        output='screen')

    node_cmd_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
                    ['xacro ', LaunchConfiguration('urdf_model')])}]
    )

    # Create the launch description and populate
    return LaunchDescription([
        arg_cmd_urdf_model,
        arg_cmd_params_file,
        node_cmd_fake_robot,
        node_cmd_robot_state_publisher
    ])
