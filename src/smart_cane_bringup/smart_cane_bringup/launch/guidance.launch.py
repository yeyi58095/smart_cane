from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



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
                FindPackageShare('smart_cane_nav'),
                'launch',
                'nav2_guidance.launch.py'
            ])
        )
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

    rviz_config = os.path.join(
        get_package_share_directory('smart_cane_bringup'),
        'rviz',
        'amcl_with_landmark_shown_or_creating.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    yolo_node = Node(
        package='smart_cane_perception',
        executable='yolo_sign_detector',
        output='screen'
    )

    ui_node = Node(
        package = 'smart_cane_nav',
        executable='qt_cmd_vel_ui',
        output='screen',
    )

    landmarkVisualizer_node = Node(
        package = 'smart_cane_landmarks',
        executable='landmark_visualizer',
        output='screen',
    )

    collision_avoiding_node = Node(
        package = 'smart_cane_nav',
        executable = 'collision_guidance',
        output = 'screen',
    )

    return LaunchDescription([
        fix_shebang,
        sim_world,
        nav2,
        yolo_node,
        rviz_node,
        ui_node,
        landmarkVisualizer_node,
        collision_avoiding_node,
    ])
