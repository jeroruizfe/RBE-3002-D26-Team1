import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz')

    slam_config_path = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'slam_config.yaml'
    )

    map_explore = Node(
        package='exploration',
        executable='map_explore.py',
        name='map_explore',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    map_client = Node(
        package='exploration',
        executable='map_client.py',
        name='map_client',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    dummy_client = Node(
        package='planning',
        executable='dummy_planner_client.py',
        name='dummy_planner_client',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        remappings=[('/move_base_simple/goal', '/nav_path/goal')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # launch map saver
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        parameters=[{'use_sim_time': True}]
    )

    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # launch SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('exploration'), 'launch'), '/slam_launch.py']),
        launch_arguments=[
            ('use_sim_time', 'False'),
            ('slam_params_file', slam_config_path)
        ]
    )
    
    # delayed launchs
    slam_delayed = TimerAction(
        period = 3.0,
        actions = [slam]
    )

    controller_delayed = TimerAction(
        period = 4.0,
        actions = [controller]
    )

    return LaunchDescription([ # return all of the launch functions
        rviz2,
        slam_delayed,
        controller_delayed,
        dummy_client,
        path_planner,
        map_explore,
        map_client,
        map_saver
    ])