from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('smart_cane_slam'), 'params', 'sync.yaml'
    ])

    node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params],
        remappings=[
            ('/scan', '/scan'),  # 如你的 topic 名稱不同，改這裡
        ],
    )
    return LaunchDescription([node])
