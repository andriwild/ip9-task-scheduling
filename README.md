# IP9 - Task Scheduling System

## Prerequisites

```sh
rosdep install --from-paths src --ignore-src -r -y
 ```


## How to build the workspace with all dependencies

This makes sure that clang can find dependencies used by ros.

```sh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
