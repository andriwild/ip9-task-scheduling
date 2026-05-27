#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source /home/andri/repos/ip9-task-scheduling/ros2_ws/install/setup.zsh

# Pick exactly one of the variants below.
# Overrides default to log_level when not set, so partial overrides work.

# Alles auf DEBUG (sehr gesprächig — gut zum Tracen)
RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
    ros2 launch event_system_bringup bringup.launch.py mode:=full \
        log_level:=DEBUG

# Engine auf DEBUG, BT-Nodes + Plugins still (kein BT-Tick-Geschwätz)
# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=DEBUG bt_log_level:=WARN plugin_log_level:=WARN

# Default INFO, nur BT-Subtree auf DEBUG (Tree-Entscheidungen mitschneiden)
# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=INFO bt_log_level:=DEBUG

# Komplett ruhig — nur WARN/ERROR
# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=WARN

# INFO global + Event-Queue Push/Pop sichtbar (Event-Flow debuggen)
# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=INFO queue_log_level:=DEBUG des.mission_manager:=DEBUG bt_log_level:=DEBUG

# DEBUG, aber Accompany-Plugin separat stumm (ad-hoc Override via --ros-args)
# RCUTILS_COLORIZED_OUTPUT=1 RCUTILS_LOGGING_MIN_SEVERITY=WARN \
#     ros2 launch event_system_bringup bringup.launch.py mode:=full \
#         log_level:=DEBUG plugin_log_level:=DEBUG \
#         --ros-args --log-level des.plugin.accompany:=WARN
