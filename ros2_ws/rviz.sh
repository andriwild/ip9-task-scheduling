#!/bin/zsh
colcon build --packages-select event_system_rviz --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
