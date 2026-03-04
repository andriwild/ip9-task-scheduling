import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    log_level = LaunchConfiguration("log_level", default="INFO")
    mode = LaunchConfiguration("mode", default="full")

    # Condition to check if mode is "headless"
    is_headless_mode = launch.substitutions.PythonExpression(["'", mode, "' == 'headless'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("log_level", default_value=log_level),
        DeclareLaunchArgument("mode", default_value=mode, description="Start mode: full or headless"),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=UnlessCondition(is_headless_mode),
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
        Node(
            package='event_system_core',
            executable='event_system_core',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            arguments=['--mode', mode],
            ros_arguments=[
                '--log-level', ['des_application:=', log_level],
                '--log-level', ['event_system_planner_node:=', log_level],
                '--log-level', ['event_system_controller_node:=', log_level],
                '--log-level', ['event_system_config_node:=', log_level],
                '--log-level', ['event_system_metrics_node:=', log_level],
                '--log-level', ['SimulationContext:=', log_level],
                '--log-level', ['Robot:=', log_level],
                '--log-level', ['State:=', log_level],
                '--log-level', ['Battery:=', log_level],
                '--log-level', ['Scheduler:=', log_level],
                '--log-level', ['Context:=', log_level],
                '--log-level', ['BT - ChargeRoutine:=', log_level]
            ]
        ),
    ])
