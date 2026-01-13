from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),

        Node(
            package='event_system_core',
            executable='event_system_core',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
