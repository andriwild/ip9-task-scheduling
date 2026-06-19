#!/bin/zsh
MAKEFLAGS="-j6" colcon build --executor sequential --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=20
