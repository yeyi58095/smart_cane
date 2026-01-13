from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup = IncludeLaunchDescription(
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
            'params_file': PathJoinSubstitution([
                FindPackageShare('smart_cane_nav'),
                'params',
                'nav2_amcl.yaml'
            ]),
        }.items()
    )

    # 把 Nav2 最終輸出 cmd_vel 改成 nav_cmd_vel
    nav2_guidance = GroupAction([
        SetRemap(src='/cmd_vel', dst='/nav_cmd_vel'),
        nav2_bringup,
    ])

    return LaunchDescription([nav2_guidance])
