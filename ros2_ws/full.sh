#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source /home/andri/repos/ip9-task-scheduling/ros2_ws/install/setup.zsh


RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
    ros2 launch event_system_bringup bringup.launch.py mode:=full \
        log_level:=DEBUG

# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=DEBUG bt_log_level:=WARN plugin_log_level:=WARN

# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=INFO bt_log_level:=DEBUG

# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=WARN

# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=INFO queue_log_level:=DEBUG des.mission_manager:=DEBUG bt_log_level:=DEBUG

# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=DEBUG plugin_log_level:=DEBUG \
#         --ros-args --log-level des.plugin.accompany:=WARN
