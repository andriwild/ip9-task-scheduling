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


# Quick Histo plotter

```sh
python3 -c "import matplotlib.pyplot as plt; import sys; data = [float(x) for x in sys.stdin]; plt.hist(data); plt.show()" < nums.txt
```

## Listen to metrics messages

```sh
ros2 topic echo /metrics_report event_system_msgs/msg/MetricsReport
```
