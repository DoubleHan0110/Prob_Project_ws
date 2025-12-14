import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    pkg_share = get_package_share_directory('model_navigation')
    
    # map file
    map_file_name = 'jackal_race.yaml'
    map_dir = os.path.join(pkg_share, 'maps', map_file_name)
    
    # config file
    params_file = os.path.join(pkg_share, 'config', 'amcl_config.yaml')
    rviz_config_file_path = os.path.join(pkg_share, 'rviz', 'amcl_local.rviz')

    # Launch para
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file_cmd = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='yaml file path')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) time if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file_path,
        description='Full path to the RVIZ config file to use')


    # file config for map server
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file_cmd,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # lifecycle sequence
    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_rviz_config_file_cmd,
        # Map Server 
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, 
                        {'yaml_filename': map_yaml_file}],
            emulate_tty=True),

        # AMCL 
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            emulate_tty=True),

        # lifecycle
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ])
