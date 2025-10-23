from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare('smart_cane_sim'), 'worlds', 'new_hospital.world']),
        description='SDF world file'
    )

    set_tb3 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'models'])
    )

    gz = ExecuteProcess(
    cmd=[
        'gazebo', '--verbose', LaunchConfiguration('world'),
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so'
    ],
    output='screen'
    )

    # ★ 改用 .urdf 而非 .xacro
    urdf_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    ])

    tb3_sdf = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'models', 'turtlebot3_burger', 'model.sdf'
    ])

    # ★ 以檔案內容提供給 robot_state_publisher（用 `cat` 讀）
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

    # ★ Gazebo 以檔案方式 spawn
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-file', tb3_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.15'   # 提高 15cm 避免卡住
        ],
        output='screen'
    )
    
    return LaunchDescription([world_arg, set_tb3, set_model_path,gz, rsp, spawn])
