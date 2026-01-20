from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -----------------------
    # Arguments (轉傳給 nav2_bringup/bringup_launch.py)
    # -----------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')

    # 你想要 remap 的輸入/輸出 topic（都可改，不寫死）
    cmd_vel_in = LaunchConfiguration('cmd_vel_in')     # nav2 原本要 publish 的
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')   # nav2 實際要 publish 到哪

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),

        # remap args
        DeclareLaunchArgument('cmd_vel_in', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_nav'),
    ]

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
            'autostart': autostart,
            'map': map_yaml,
        }.items()
    )

    # ✅ 關鍵：把 include 包進 GroupAction，套用 SetRemap
    # 這樣 bringup_launch.py 裡面啟動的所有 node，只要用到 /cmd_vel 都會被 remap。
    nav2_with_remap = GroupAction([
        SetRemap(src=cmd_vel_in, dst=cmd_vel_out),
        nav2_bringup_launch,
    ])

    return LaunchDescription(
        declare_args + [
            nav2_with_remap
        ]
    )
