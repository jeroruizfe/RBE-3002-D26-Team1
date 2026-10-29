import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():   
    os.environ['MESA_GL_VERSION_OVERRIDE'] = '4.5'
    os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '450'
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    run_mode =  DeclareLaunchArgument('sim_mode', default_value='True', # Defines a launch argument for selecting Gazebo or robot
                                  description='Run Gazebo or robot')
    mode = LaunchConfiguration('sim_mode') # Makes the command line agrument

    rviz_config_dir = os.path.join(
        get_package_share_directory('control'),
        'rviz',
        'control_pkg.rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py']),
        launch_arguments = [
            ('use_sim_time', 'True')
        ],
        condition=IfCondition(mode) # Only launch Gazebo if the sim_mode argument is set to True
    )

    rviz2 =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    static_transform = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"] 
            )

    # Controller node
    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        # Change the parameter to use real time or simulated time (real vs sim)
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Path Generator node
    path_generator = Node(
        package='control',
        executable='path_generator.py',
        name='path_generator',
        output='screen'
    )

    # Delayed path_generator
    delayed_path_generator = TimerAction(
        period=5.0, # Delays 5 seconds before starting the path generator
        actions=[path_generator] # Starts the path generator
    )

    return LaunchDescription([
        run_mode,
        gz_sim,
        rviz2,
        static_transform,
        controller,
        # delayed_path_generator
    ])
