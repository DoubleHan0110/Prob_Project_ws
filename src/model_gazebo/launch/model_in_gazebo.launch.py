from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

import xacro


import os

def generate_launch_description():
    # ====== path settings ======
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    model_gazebo_pkg = FindPackageShare('model_gazebo').find('model_gazebo')
    
    gazebo_launch = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gazebo.launch.py']
    )

    description_pkg = FindPackageShare('model_description').find('model_description')

    # world file name
    default_world = PathJoinSubstitution(
        [model_gazebo_pkg, 'worlds', 'jackal_race.world']
    )

    controller_config = os.path.join(
        model_gazebo_pkg,
        'config',
        'controllers.yaml')

    # urdf file name
    model_name = 'LeKiwi_simplified_lidar'                             
    default_model_path = os.path.join(
        description_pkg, 'urdf', model_name + '.urdf'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World file'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to robot urdf/xacro'
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
    )

    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )

    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    omni_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller", "--controller-manager", "/controller_manager"],
    )

    vel_cmd_trans_to_wheel = Node(
        package='model_gazebo',
        executable='vel_cmd_to_wheel',
        name='vel_cmd_to_wheel',
        output='screen',
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_separation': 0.149}
        ]
    )

    ground_truth_pub_node = Node(
        package='model_gazebo',
        executable='ground_truth_pub',
        name='ground_truth_pub',
        output='screen'
    )

    ukf_node = Node(
        package='model_gazebo',
        executable='ukf_odometry',
        name='ukf_odometry',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        gazebo,
        rsp_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        omni_wheel_controller_spawner,
        vel_cmd_trans_to_wheel,
        ground_truth_pub_node,
        ukf_node,
    ])
