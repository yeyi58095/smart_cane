'''# 
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
'''

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # -------------------------
    # Launch args
    # -------------------------
    mux_out_arg = DeclareLaunchArgument(
        'mux_out',
        default_value='/cmd_vel',
        description='Final cmd_vel output topic from twist_mux'
    )

    nav_cmd_arg = DeclareLaunchArgument(
        'nav_cmd',
        default_value='/cmd_vel_nav',
        description='Nav2 cmd_vel output topic (input to mux)'
    )

    align_cmd_arg = DeclareLaunchArgument(
        'align_cmd',
        default_value='/cmd_vel_align',
        description='Align-to-target cmd_vel output topic (input to mux)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/tb3/camera/image_raw',
        description='Camera image topic for align_to_target'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=os.path.expanduser('~/smart_cane/yolov8n.pt'),
        description='YOLO model path for align_to_target'
    )

    # -------------------------
    # World
    # -------------------------
    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('smart_cane_sim'),
                'launch',
                'sim_world.launch.py'
            ])
        )
    )

    # -------------------------
    # Nav2 (remap /cmd_vel -> /cmd_vel_nav)
    # -------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('smart_cane_bringup'),
                'launch',
                'nav2_bringup_remap.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'False',
            'autostart': 'True',
            'map': '/home/daniel/smart_cane/maps/new_hospital_map.yaml',
            'cmd_vel_in': '/cmd_vel',
            'cmd_vel_out': LaunchConfiguration('nav_cmd'),
        }.items()
    )

    # -------------------------
    # Fix shebang for YOLO in perception package (optional)
    # -------------------------
    fix_shebang = ExecuteProcess(
        cmd=['ros2', 'run', 'smart_cane_perception', 'fix_yolo_shebang'],
        output='screen'
    )

    # -------------------------
    # Twist mux (nav vs align -> mux_out)
    # -------------------------
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

    # -------------------------
    # Align-to-target (ALWAYS ON, controlled by /align/enable + /align/target_class)
    # -------------------------
    align_node = Node(
        package='smart_cane_nav',
        executable='align_to_target',
        name='align_to_target',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'cmd_topic': LaunchConfiguration('align_cmd'),
            'yolo_model': LaunchConfiguration('yolo_model'),

            # 你想要的：看到就算有看到（>=0.4）
            'detect_th': 0.4,
            'control_th': 0.6,
        }]
    )

    # -------------------------
    # RViz
    # -------------------------
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

    return LaunchDescription([
        mux_out_arg,
        nav_cmd_arg,
        align_cmd_arg,
        image_topic_arg,
        yolo_model_arg,

        fix_shebang,
        sim_world,
        nav2,
        twist_mux_node,
        align_node,
        rviz_node,
    ])
