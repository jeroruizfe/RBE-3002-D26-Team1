import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    exploration_dir = get_package_share_directory('exploration')
    planning_dir = get_package_share_directory('planning')

    rviz_config_path = os.path.join(planning_dir, 'rviz', 'lab3_config.rviz')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_dir, 'launch', 'localization.launch.py')
        ),
    )

    navigator = Node(
        package='exploration',
        executable='navigator.py',
        name='navigator',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    goal_sender = Node(
        package='exploration',
        executable='goal_sender.py',
        name='goal_sender',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        parameters=[{'use_sim_time': False}],
        output='screen',
        remappings=[
            ('/move_base_simple/goal', '/move_base_simple/goal_disabled'),
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    particle_relay = Node(
        package='exploration',
        executable='particle_rviz_relay.py',
        name='particle_rviz_relay',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Navigator starts after 6s to give AMCL/map_server time to come up
    navigator_delayed = TimerAction(period=6.0, actions=[navigator])

    return LaunchDescription([
        rviz2,
        controller,
        particle_relay,
        localization,
        navigator_delayed,
        path_planner,
        goal_sender
    ])