from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command

import os

def generate_launch_description():
    # ====== 路径设置 ======
    # 找到 gazebo_ros 自带的 gazebo.launch.py
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    world_pkg = FindPackageShare('model_gazebo').find('model_gazebo')
    
    gazebo_launch = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gazebo.launch.py']
    )

    # 找到你自己的 model_description 包
    desc_pkg = FindPackageShare('model_description').find('model_description')

    default_world = PathJoinSubstitution(
        [world_pkg, 'worlds', 'turtlebot3_house.world']
    )

    # 机器人 URDF / Xacro 路径（改成你自己的文件名）
    model_name = 'LeKiwi'                             #urdf文件名
    default_model_path = PathJoinSubstitution(
        [desc_pkg, 'urdf', model_name + '.urdf']  
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

    return LaunchDescription([
        world_arg,
        model_arg,
        gazebo,
        rsp_node,
        spawn_entity
    ])
