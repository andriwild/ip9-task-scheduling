#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
WS_DIR=${0:A:h}
source $WS_DIR/install/setup.zsh
RCUTILS_COLORIZED_OUTPUT=1 \
RCUTILS_LOGGING_MIN_SEVERITY=INFO \
ros2 launch event_system_bringup bringup.launch.py mode:=build log_level:=INFO
