# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Build (from workspace root)
colcon build --packages-select tsm_gz_sim tsm
source install/setup.bash

# Launch simulation
ros2 launch tsm_gz_sim bringup.launch.py
ros2 launch tsm_gz_sim bringup.launch.py tube_type:=straight  # explicit default
```

No test suite yet.

## Architecture

ROS 2 (ament_cmake) + Gazebo Harmonic simulation for measuring steel tube straightness. Two packages:

### `tsm_gz_sim` — simulation driver

- `launch/bringup.launch.py` — starts Gazebo Harmonic with `measure_world.sdf`, spawns `straight_tube`, starts `ros_gz_bridge`, two static TF publishers (rgbd_1 and rgbd_2 cameras), RViz2, and `tsm_gz_sim_node`
- `sdf/measure_world.sdf` — Gazebo world
- `sdf/straight_tube/straight_tube.sdf` — tube model
- `config/topic_bridge_config.yaml` — ROS↔Gazebo topic bridge config
- `config/sim_config.yaml` — params for `tsm_gz_sim_node`

**`TsmGzSimNode`** (`tsm_gz_sim_node`): moves tube along X axis in Gazebo via `/world/measure_world/set_pose` gz-transport service; broadcasts `world→tube` TF. Subscribes to `tube_cmd` (String: `f`/`b`/`s`).

**`KeyboardNode`** (`keyboard_node`): reads stdin in raw mode, publishes `f`/`b`/`s` to `tube_cmd`.

### `tsm` — measurement core

**`TsmNode`** (`tsm_node`): subscribes to two synced RGBD point clouds (`rgbd_1/points`, `rgbd_2/points`), merges them in world frame, crops to configurable box, runs PCA to find tube axis, rotates cloud to align axis with X, splits into 3 segments (start/middle/end), publishes merged + 3 segment clouds. Step 7 (fitting/straightness computation) not yet implemented.

Key params (`tsm` package):
- `valid_pc_area.{x,y,z}_{min,max}` — crop box around tube (default ±0.5 m)
- `cutting_fittting.length_of_each_segment` — segment length along tube axis (default 0.1 m)

Camera TF frames (set in launch):
- `rgbd_1/camera_link/rgbd_1` at (0, +1, +1) looking down-inward
- `rgbd_2/camera_link/rgbd_2` at (0, −1, +1) looking down-inward

## C++ Tooling

**clangd** (`.clangd`): C++20, compilation database at `build/` — run `colcon build` first to generate it.

**clang-format** (`.clang-format`): Google base, 2-space indent, 80-col limit, `BreakBeforeBraces: Custom`, `PointerAlignment: Left`, `SortIncludes: false`.
