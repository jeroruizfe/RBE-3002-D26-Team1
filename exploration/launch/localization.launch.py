import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    maps_dir = os.path.join(
        get_package_share_directory('exploration'),
        'maps'
    )
    map_yaml = os.path.join(maps_dir, 'explored_map.yaml')

    amcl_config = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'amcl_config.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': map_yaml},
            {'map_file_path': maps_dir},   # sets working dir for relative pgm path
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': False}
        ],
        remappings=[('scan', '/scan')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    particle_relay = Node(
        package='exploration',
        executable='particle_rviz_relay.py',
        name='particle_rviz_relay',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        map_server, 
        amcl, 
        lifecycle_manager,
        # particle_relay
    ])