import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    log_level = LaunchConfiguration("log_level", default="INFO")
    # Per-subtree overrides — default to log_level if not set explicitly.
    bt_log_level     = LaunchConfiguration("bt_log_level",     default=log_level)
    plugin_log_level = LaunchConfiguration("plugin_log_level", default=log_level)
    io_log_level     = LaunchConfiguration("io_log_level",     default=log_level)
    # Event-queue push/pop traffic — opt-in DEBUG, otherwise stays at log_level.
    queue_log_level  = LaunchConfiguration("queue_log_level",  default=log_level)
    mode = LaunchConfiguration("mode", default="full")
    rounds = LaunchConfiguration("rounds", default="1")

    # RViz only in the interactive full mode (not headless, not the build tool).
    is_full_mode = launch.substitutions.PythonExpression(["'", mode, "' == 'full'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("log_level",        default_value=log_level),
        DeclareLaunchArgument("bt_log_level",     default_value=bt_log_level,
                              description="Override level for des.bt.* (default: log_level)"),
        DeclareLaunchArgument("plugin_log_level", default_value=plugin_log_level,
                              description="Override level for des.plugin.* (default: log_level)"),
        DeclareLaunchArgument("io_log_level",     default_value=io_log_level,
                              description="Override level for des.io.* (default: log_level)"),
        DeclareLaunchArgument("queue_log_level",  default_value=queue_log_level,
                              description="Override level for des.event_queue (default: log_level). Set to DEBUG to see push/pop traffic."),
        DeclareLaunchArgument("mode", default_value=mode, description="Start mode: full, headless or build"),
        DeclareLaunchArgument("rounds", default_value=rounds, description="Number of rounds in headless mode"),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(is_full_mode),
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
            arguments=['--mode', mode, '--rounds', rounds],
            ros_arguments=[
                # All DES-internal loggers live under the `des.*` hierarchy,
                # so one switch covers the whole engine. More specific entries
                # override the root for their subtree.
                '--log-level', ['des:=',             log_level],
                '--log-level', ['des.bt:=',          bt_log_level],
                '--log-level', ['des.plugin:=',      plugin_log_level],
                '--log-level', ['des.io:=',          io_log_level],
                '--log-level', ['des.event_queue:=', queue_log_level],
                # The standalone ROS nodes have their own logger names.
                '--log-level', ['event_system_planner_node:=',    log_level],
                '--log-level', ['event_system_controller_node:=', log_level],
                '--log-level', ['event_system_config_node:=',     log_level],
                '--log-level', ['event_system_metrics_node:=',    log_level],
            ]
        ),
    ])
