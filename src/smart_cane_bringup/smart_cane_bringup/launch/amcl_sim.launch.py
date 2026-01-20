from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mux_out_arg = DeclareLaunchArgument(
        'mux_out',
        default_value='/cmd_vel',
        description='Final cmd_vel output topic from twist_mux'
    )

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('smart_cane_sim'),'launch','sim_world.launch.py'])
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('smart_cane_bringup'),'launch','nav2_bringup_remap.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'False',
            'autostart': 'True',
            'map': '/home/daniel/smart_cane/maps/new_hospital_map.yaml',
            'cmd_vel_in': '/cmd_vel',
            'cmd_vel_out': '/cmd_vel_nav',
        }.items()
    )

    fix_shebang = ExecuteProcess(
        cmd=['ros2','run','smart_cane_perception','fix_yolo_shebang'],
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('smart_cane_bringup'),
        'rviz',
        'amcl_with_landmark_shown_or_creating.rviz'
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen'
    )

    twist_mux_yaml = os.path.join(
        get_package_share_directory('smart_cane_bringup'),
        'config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml],
        remappings=[
            ('cmd_vel_out', LaunchConfiguration('mux_out')),
        ]
    )


    return LaunchDescription([
        mux_out_arg,
        fix_shebang,
        sim_world,
        nav2,
        twist_mux_node,
        rviz_node,
    ])
