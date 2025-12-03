from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    # 選擇 world 的參數（跟原本一樣）
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('smart_cane_sim'),
            'worlds',
            'new_hospital.world'
        ]),
        description='SDF world file'
    )

    # TurtleBot3 型號
    set_tb3 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # ✅ 把 GAZEBO_MODEL_PATH 指到自己的 models
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([
            FindPackageShare('smart_cane_sim'),
            'models'
        ])
    )

    # 啟動 Gazebo，載入 world
    gz = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', LaunchConfiguration('world'),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # ✅ URDF 改成吃自己 package 底下的 urdf 檔
    urdf_path = PathJoinSubstitution([
        FindPackageShare('smart_cane_sim'),
        'urdf',
        'turtlebot3_burger.urdf'
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name='cat'), ' ', urdf_path]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ✅ Gazebo spawn 改成自己 package 的 SDF
    tb3_sdf = PathJoinSubstitution([
        FindPackageShare('smart_cane_sim'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    ])

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-file', tb3_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.15'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        set_tb3,
        set_model_path,
        gz,
        rsp,
        spawn
    ])
