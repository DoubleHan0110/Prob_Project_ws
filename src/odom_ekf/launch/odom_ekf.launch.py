from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    odom_ekf_pkg = FindPackageShare('odom_ekf').find('odom_ekf')

    odom_ekf_node = Node(
        package='odom_ekf',
        executable='odom_ekf',
        name='odom_ekf'
    )






    return LaunchDescription([
        odom_ekf_node
    ])