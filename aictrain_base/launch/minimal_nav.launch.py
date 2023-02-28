import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


AIC_MODEL = os.environ['AIC_MODEL']


def generate_launch_description():
    # Get the launch directory
    pkg_this_project = get_package_share_directory('aictrain_base')
    pkg_aicmodel = get_package_share_directory(AIC_MODEL)

    model_params_file = LaunchConfiguration('model_params_file')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'bt_navigator',
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'smoother_server',
        'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    arg_cmd_model_params_file = DeclareLaunchArgument(
        'model_params_file',
        default_value=os.path.join(pkg_aicmodel, 'config', 'robot_model.yaml'),
        description='AIC robot model package params yaml path')

    arg_cmd_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    arg_cmd_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_this_project, 'maps', 'rectangle', 'map.yaml'),
        description='Full path to map yaml file to load')

    arg_cmd_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    arg_cmd_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_this_project, 'params', 'basic_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    arg_cmd_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    arg_cmd_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    arg_cmd_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        actions=[
            Node(package='tf2_ros',
                 executable='static_transform_publisher',
                 arguments='0 0 0 0 0 0 map odom'.split(' ')),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, model_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    # Create the launch description and populate
    return LaunchDescription([
        stdout_linebuf_envvar,
        arg_cmd_model_params_file,
        arg_cmd_namespace,
        arg_cmd_map_yaml,
        arg_cmd_use_sim_time,
        arg_cmd_params_file,
        arg_cmd_autostart,
        arg_cmd_use_respawn,
        arg_cmd_log_level,
        load_nodes
    ])
