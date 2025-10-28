from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params = PathJoinSubstitution([FindPackageShare('smart_cane_nav'), 'params', 'nav2_slam.yaml'])
    default_bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_timeouts.xml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('default_bt_xml', default_value=default_bt_xml),

        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[params, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel', '/cmd_vel')]),

        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params, {'use_sim_time': use_sim_time,
                                                   'default_bt_xml_filename': default_bt_xml}]),

        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server',
             output='screen', parameters=[params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': True,
                          'node_names': ['controller_server','planner_server','bt_navigator','recoveries_server']}]),
    ])
