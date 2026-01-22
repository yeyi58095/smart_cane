from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # -------------------------
    # Args
    # -------------------------
    mux_out_arg = DeclareLaunchArgument(
        'mux_out',
        default_value='/nav_cmd_vel',
        description='Final suggested cmd_vel output topic for external controller'
    )

    nav_cmd_arg = DeclareLaunchArgument(
        'nav_cmd',
        default_value='/cmd_vel_nav',
        description='Nav2 cmd_vel output topic (input to mux)'
    )

    align_cmd_raw_arg = DeclareLaunchArgument(
        'align_cmd_raw',
        default_value='/cmd_vel_align',
        description='Align cmd_vel raw output (before safety filter)'
    )

    align_cmd_safe_arg = DeclareLaunchArgument(
        'align_cmd_safe',
        default_value='/cmd_vel_align_safe',
        description='Align cmd_vel after safety filter (input to mux)'
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
    # Nav2: output to /cmd_vel_nav (NOT /cmd_vel)
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

            # keep interface, but actual output is cmd_vel_out
            'cmd_vel_in': '/cmd_vel',
            'cmd_vel_out': LaunchConfiguration('nav_cmd'),
        }.items()
    )

    # -------------------------
    # Fix YOLO shebang (optional)
    # -------------------------
    fix_shebang = ExecuteProcess(
        cmd=['ros2', 'run', 'smart_cane_perception', 'fix_yolo_shebang'],
        output='screen'
    )

    # -------------------------
    # Align-to-target (publishes raw align cmd)
    # -------------------------
    align_node = Node(
        package='smart_cane_nav',
        executable='align_to_target',
        name='align_to_target',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'cmd_topic': LaunchConfiguration('align_cmd_raw'),
            'yolo_model': LaunchConfiguration('yolo_model'),

            # 你之前的設定保留
            'control_th': 0.6,
        }]
    )

    # -------------------------
    # Safety filter: /cmd_vel_align -> /cmd_vel_align_safe
    # -------------------------
    safety_node = Node(
        package='smart_cane_nav',
        executable='cmd_vel_safety_filter',
        name='cmd_vel_safety_filter',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'in_cmd_topic': LaunchConfiguration('align_cmd_raw'),
            'out_cmd_topic': LaunchConfiguration('align_cmd_safe'),

            'front_half_angle_deg': 15.0,
            'stop_dist': 0.25,

            'scan_timeout': 0.6,
            'min_valid_margin': 0.02,
            'log_every_sec': 1.0,
        }]
    )

    # -------------------------
    # Twist mux: nav vs align_safe -> mux_out (/nav_cmd_vel)
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
    # RViz (optional)
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

    # -------------------------
    # UI (external controller view / manual apply)
    # -------------------------
    ui_node = Node(
        package='smart_cane_nav',
        executable='qt_cmd_vel_ui',
        name='qt_cmd_vel_ui',
        output='screen',
        # 如果你的 UI 內部是訂閱 /nav_cmd_vel 來顯示箭頭，那就不用 remap
        # 若 UI 預設訂閱 /cmd_vel，請打開下面 remappings
        # remappings=[('/cmd_vel', LaunchConfiguration('mux_out'))]
    )

    # -------------------------
    # Landmark visualizer (optional)
    # -------------------------
    landmark_visualizer = Node(
        package='smart_cane_landmarks',
        executable='landmark_visualizer',
        name='landmark_visualizer',
        output='screen'
    )

    collision_avoiding_node = Node(
        package = 'smart_cane_nav',
        executable = 'collision_guidance',
        output = 'screen',
    )


    return LaunchDescription([
        mux_out_arg,
        nav_cmd_arg,
        align_cmd_raw_arg,
        align_cmd_safe_arg,
        image_topic_arg,
        yolo_model_arg,

        fix_shebang,
        sim_world,
        nav2,

        align_node,
        safety_node,
        twist_mux_node,

        rviz_node,
        ui_node,
        landmark_visualizer,
        collision_avoiding_node,
    ])
