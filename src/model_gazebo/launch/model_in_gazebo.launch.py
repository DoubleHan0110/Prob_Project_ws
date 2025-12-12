from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command

import os

def generate_launch_description():
    # ====== path settings ======
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    model_gazebo_pkg = FindPackageShare('model_gazebo').find('model_gazebo')
    
    gazebo_launch = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gazebo.launch.py']
    )

    description_pkg = FindPackageShare('model_description').find('model_description')
    default_world = PathJoinSubstitution(
        [model_gazebo_pkg, 'worlds', 'empty_world.world']
    )
    # default_world = PathJoinSubstitution(
    #     [gazebo_pkg, 'worlds', 'empty.world']
    # )

    controller_config = os.path.join(
        model_gazebo_pkg,
        'config',
        'controllers.yaml'
    )

    # 机器人 URDF / Xacro 路径（改成你自己的文件名）
    model_name = 'LeKiwi_simplified'                             #urdf文件名
    default_model_path = PathJoinSubstitution(
        [description_pkg, 'urdf', model_name + '.urdf']  
    )

    # ====== 声明 launch 参数（可选） ======
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

    # ====== 启动 Gazebo（server + client） ======
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # ====== 生成 robot_description 内容（用 xacro 也能处理 urdf） ======
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    # ====== robot_state_publisher，用于 TF + /robot_description ======
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # ====== 用 spawn_entity.py 把机器人丢进 Gazebo ======
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-topic', 'robot_description',
            # '-file', default_model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )

    # 1. 启动 Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 3. 启动 Omni Wheel Controller
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
        # change the default para here
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_separation': 0.132}
        ]
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
    ])
