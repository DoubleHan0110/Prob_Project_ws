from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    odom_ekf_pkg = FindPackageShare('odom_ekf').find('odom_ekf')

    odom_ekf_node = Node(
        package='odom_ekf',
        executable='odom_ekf',
        name='odom_ekf',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )






    return LaunchDescription([
        odom_ekf_node
    ])