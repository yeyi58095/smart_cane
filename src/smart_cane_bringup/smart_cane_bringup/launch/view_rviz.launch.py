from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    cfg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('smart_cane_bringup'), 'rviz', 'sim_view_default.rviz'])
    )
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', LaunchConfiguration('rviz_config')], output='screen')
    return LaunchDescription([cfg, rviz])
