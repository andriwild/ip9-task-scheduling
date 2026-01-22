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

## TF Transform

```sh
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

## Environments Vars

```sh
export RCUTILS_COLORIZED_OUTPUT=1
```
