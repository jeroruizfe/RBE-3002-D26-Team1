import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions

def generate_launch_description():

    # map file path for simple_map
    # map_file_path = os.path.join(
    #     get_package_share_directory('planning'), 
    #     'maps',
    #     'simple_map.yaml'
    # )


    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py']),
        launch_arguments = [
            ('use_sim_time', 'True')
        ],
        # condition=IfCondition(mode) # Only launch Gazebo if the sim_mode argument is set to True
    )

    # map_server node
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
        # parameters=[{'yaml_filename':map_file_path}]
    # )

    # path planner client node
    dummy_client = Node(
        package='planning',
        executable='dummy_planner_client.py',
        name='dummy_planner_client',
        parameters=[{'use_real_time': False}],
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

    # start the node lifecyle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_real_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    ) 

    # map to odom static transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"],
        output='screen'
    )


    # odom to base_link transform, remove if running sim or real robot
    static_tf_fake_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_robot',
        arguments=["0.5", "0.5", "0", "0.", "0", "0", "odom", "base_link"],
        output='screen'
    )



    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    reset_simulation = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
            )
        ]
    )

    return LaunchDescription([ # return all of the launch functions
        # map_server,
        lifecycle_manager, 
        static_tf_fake_robot,
        static_tf,
        rviz2,
        gz_sim,
        dummy_client,
        path_planner,
        controller,
        reset_simulation
    ])