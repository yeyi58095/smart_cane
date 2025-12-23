from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('smart_cane_sim'),
                'launch',
                'sim_world.launch.py'
            ])
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'False',
            'autostart': 'True',
            'map': '/home/daniel/smart_cane/maps/new_hospital_map.yaml',
        }.items()
    )

    initialpose = Node(
        package='smart_cane_bringup',
        executable='initialpose_pub',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    fix_shebang = ExecuteProcess(
        cmd=[
            'ros2', 'run',
            'smart_cane_perception', 'fix_yolo_shebang'
        ],
        output='screen'
    )

    yolo_node = Node(
        package='smart_cane_perception',
        executable='yolo_sign_detector',
        output='screen'
    )

    return LaunchDescription([
        fix_shebang,
        sim_world,
        nav2,
        initialpose,
        yolo_node,
    ])
