#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source /home/andri/repos/ip9-task-scheduling/ros2_ws/install/setup.zsh
RCUTILS_COLORIZED_OUTPUT=1 \
RCUTILS_LOGGING_MIN_SEVERITY=WARN \
ros2 launch event_system_bringup bringup.launch.py log_level:=DEBUG mode:=full
