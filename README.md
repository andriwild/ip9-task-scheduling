# IP9 - Task Scheduling System

<p align="center">
  <img src="screenshots/rviz_1.png" width="49%" alt="RViz panel" />
  <img src="screenshots/rviz_2.png" width="49%" alt="RViz panel" />
</p>

![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-colcon-064F8C?logo=cmake&logoColor=white)
![PostGIS](https://img.shields.io/badge/PostGIS-336791?logo=postgresql&logoColor=white)

A discrete event simulation (DES) for evaluating task scheduling strategies of
an autonomous service robot at FHNW. The simulator drives a behaviour-tree
controlled robot through stochastically generated missions (person arrivals,
conversations, accompany requests, search routines) while tracking battery
state and charging behaviour. The system is built on ROS 2 and can run either
headlessly for batch experiments or interactively with an RViz panel for
inspection and control.

## Key Components

- **Discrete Event System** — A time-sorted priority `EventQueue` drives the
  simulation. `SimulationContext` coordinates events, robot state and the
  scheduler. See `ros2_ws/src/event_system_core/src/model` and
  `ros2_ws/src/event_system_core/src/sim`.
- **ROS 2** — The core is packaged as ROS 2 nodes under `ros2_ws/src`:
  `event_system_core`, `event_system_bringup`, `event_system_msgs`,
  `event_system_rviz` and `event_system_tf_transform`. Metrics are published on
  `/metrics_report` (`event_system_msgs/msg/MetricsReport`).
- **RViz Panel** — `event_system_rviz` provides several custom RViz panel
  plugins (`DesPanel`, `DesSystemConfig`, `DesTimelinePanel`,
  `DesMetricsPanel`, `DesOccupancyPanel`) to start, pause, configure and
  inspect the simulation visually.
- **Stochastik** — Random distributions for event timing, person behaviour and
  mission generation live in `ros2_ws/src/event_system_core/src/util/rnd.h`.
- **Energiemanagement** — The robot model in `model/battery.hpp`, `robot.h`
  and the `charge` behaviour (`behaviour/charge.h`) simulate battery drain,
  charging and the decision logic for returning to the dock.
- **Roboter / Behaviour Tree** — Robot behaviours are implemented as a BT in
  `event_system_core/src/behaviour` (`idle`, `search`, `accompany`, `charge`,
  `conversation`, `mission_control`).
- **Blender** — The building geometry and waypoints are modelled in a Blender
  file (`fhnw.blend`) and exported to the PostGIS database `wsr` by Python
  scripts under `blender/`:
  - `room_poly_to_db.py` — exports the `RoomPolygons` collection as room
    polygons.
  - `waypoints_to_db.py` — exports the `Waypoints` collection as points of
    interest (position + yaw).
  - `install_deps.py` — installs `sqlalchemy` / `geoalchemy2` into Blender's
    bundled Python.

## Prerequisites

```sh
rosdep install --from-paths src --ignore-src -r -y
```

A PostgreSQL/PostGIS database reachable at
`postgresql://wsr_user:wsr_password@localhost:5432/wsr` is required for map
and waypoint data.

## Build the Workspace

Exporting compile commands makes sure clang tooling can find the ROS
dependencies.

```sh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Running

### 1. Start the navigation / map backend

`planner.sh` sources ROS 2 Jazzy and the `fhnw-dev-workspace`, then launches
the map server and planner from `fhnw_bot_navigation` via `des.sh`. This must
be running before the DES is started.

```sh
./planner.sh
```

### 2. Launch the DES

```sh
ros2 launch event_system_bringup bringup.launch.py
```

Launch arguments:

- `mode:=full` (default) or `mode:=headless` — `full` starts RViz, `headless`
  skips it for batch runs.
- `rounds:=N` — number of simulation rounds in headless mode.
- `use_sim_time:=true` (default).
- `log_level:=INFO` — log level applied to all simulation nodes.

Example headless batch run:

```sh
ros2 launch event_system_bringup bringup.launch.py mode:=headless rounds:=50 log_level:=WARN
```

### 3. RViz Panels

When launched in `mode:=full`, RViz 2 is started automatically by
`bringup.launch.py`. To enable the custom DES panels in RViz, use
`Panels -> Add New Panel` and add any of:

- `des_panel/DesPanel` — main simulation control (start / pause / step).
- `des_system_config/DesSystemConfig` — runtime configuration.
- `des_timeline_panel/DesTimelinePanel` — event timeline view.
- `des_metrics_panel/DesMetricsPanel` — live metrics.
- `des_occupancy_panel/DesOccupancyPanel` — occupancy / map view.

The panel layout can be saved into an RViz config so it is restored on
subsequent launches.

### 4. TF Transform (optional)

```sh
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

### Exporting Blender data to the database

Open the Blender scene and run the scripts from `blender/` in Blender's text
editor (once, to populate the `wsr` database):

```text
blender/install_deps.py      # installs sqlalchemy / geoalchemy2 in Blender
blender/room_poly_to_db.py   # exports RoomPolygons collection
blender/waypoints_to_db.py   # exports Waypoints collection
```

## Environment Variables

```sh
export RCUTILS_COLORIZED_OUTPUT=1
```

## Inspecting Metrics

```sh
ros2 topic echo /metrics_report event_system_msgs/msg/MetricsReport
```

## Quick Histogram Plotter

```sh
python3 -c "import matplotlib.pyplot as plt; import sys; data = [float(x) for x in sys.stdin]; plt.hist(data); plt.show()" < nums.txt
```
