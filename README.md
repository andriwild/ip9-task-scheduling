# IP9 - Task Scheduling System

<p align="center">
  <img src="screenshots/sim_rviz.png" width="90%" alt="Simulation in RViz" />
</p>

![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-colcon-064F8C?logo=cmake&logoColor=white)

A discrete event simulation of an autonomous service robot at FHNW. A behaviour
tree decides what the robot does next, the simulation tracks missions, people,
battery and charging over a building map. It runs either headless or
interactively with RViz panels.

The workspace lives in `ros2_ws/`, all commands below are run from there.

## Setup

### Headless, without ROS

The simulator builds standalone into `build-headless/`. Nothing but a C++23
compiler and CMake is needed.

```sh
./build_headless.sh          # Release build
./build_headless.sh --clean  # from scratch
```

### With ROS

Needs ROS 2 Jazzy plus the FHNW navigation stack for path planning.

```sh
rosdep install --from-paths src --ignore-src -r -y
./build.sh                   # colcon, C++23, exports compile_commands.json
```

`planner.sh` sits in the repository root, sources ROS 2 and the
`fhnw-dev-workspace` and starts map server and planner. It has to run in its
own terminal before the interactive mode is started.

```sh
../planner.sh
```

A PostgreSQL/PostGIS database at
`postgresql://wsr_user:wsr_password@localhost:5432/wsr` holds the building
geometry. It is only read by the offline export tools, a normal run works from
`config/building.json`.

## Running

### Headless

```sh
./build-headless/des_headless --mode headless \
    --config overrides/thesis/T1_energiereserve/1_next_mission.json \
    --rounds 20 --log-level des:=WARN
```

### With GUI

```sh
./full.sh                                  # simulation plus RViz
./full.sh overrides/thesis/T16_termintreue/gui.json   # with an override
```

Under the hood this is a launch file, so the same run works directly:

```sh
ros2 launch event_system_bringup bringup.launch.py mode:=full \
    config:=overrides/thesis/T16_termintreue/gui.json
```

The RViz panels are added through `Panels -> Add New Panel`: `DesPanel` for
start, pause and step, `DesSystemConfig` for the parameters, plus
`DesTimelinePanel`, `DesMetricsPanel` and `DesOccupancyPanel`.

## Options

| Option | Launch argument | Meaning |
|---|---|---|
| `--config <file>` | `config:=` | Override merged onto the base config |
| `--base-config <file>` | `base_config:=` | Replaces the base config, default `config/default/sim_config.json` |
| `--rounds N` | `rounds:=` | Number of rounds, each with its own seed |
| `--out-dir <dir>` | `out_dir:=` | Where metrics and traces are written |
| `--run-id <id>` | `run_id:=` | Name prefix of the output files |
| `--log-level des:=WARN` | `log_level:=` | Log level, per subtree with `des.bt`, `des.plugin`, `des.io` |

Paths inside a config are resolved relative to `config/`.

## Configuration

Everything is JSON below `config/`:

```
config/
├── default/     sim_config.json and sim_config_gui.json, the merge bases
├── overrides/   partial configs, merged onto a base with --config
├── scenarios/   the mission sets
├── employees/   person populations
└── building.json, tours/
```

The scenario is not a command line option, it is the `scenario_path` inside
the config:

```json
{
    "scenario_path": "scenarios/thesis_baseline.json",
    "employees_path": "employees/employee_gebaeude84.json",
    "sim_duration": 2592000
}
```

A scenario file holds three lists: `orders` are appointments with a fixed time,
`background` are jobs the robot fills its idle time with, and
`ad_hoc_generators` produce unannounced requests at a given rate. Person names
in a scenario must exist in the population named by `employees_path`.

## Tests

```sh
./run_tests.sh
```
