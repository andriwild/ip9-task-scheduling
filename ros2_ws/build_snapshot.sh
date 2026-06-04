#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source /home/andri/repos/ip9-task-scheduling/ros2_ws/install/setup.zsh
RCUTILS_COLORIZED_OUTPUT=1 \
RCUTILS_LOGGING_MIN_SEVERITY=INFO \
ros2 launch event_system_bringup bringup.launch.py mode:=build log_level:=INFO
