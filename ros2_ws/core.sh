#!/bin/zsh
colcon build --packages-select event_system_core --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=20
