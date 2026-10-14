# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Rolling-Bonxai is a C++17 ROS 2 (colcon) workspace that adds a chunked, rolling-window layer on top of [Bonxai](https://github.com/facontidavide/Bonxai) sparse voxel occupancy grids. Only chunks near the robot stay in memory; the rest are serialized to disk and reloaded per a pluggable policy. The repo root is a set of sibling ROS packages (it is expected to live in a colcon workspace's `src/`; the devcontainers in `.devcontainer/` + `.docker/` mount it at `/ros2_ws/src` for Humble and Jazzy).

## Build / test / run

```bash
# from the colcon workspace root (not this repo's root)
colcon build --packages-select bonxai_core bonxai_map rolling_bonxai rolling_bonxai_msgs rolling_bonxai_ros
source install/setup.bash

# tests (gtest via ament_cmake_gtest; binaries: test_common, test_coordinate_system,
# test_file_storage_backend, test_chunk_storage, test_chunk_loading_policy, test_transition_manager)
colcon test --packages-select rolling_bonxai --event-handlers console_direct+
# single test: run the built binary with a gtest filter
./build/rolling_bonxai/test_transition_manager --gtest_filter='Suite.Case'

# benchmarks (off by default; nanobench, see rolling_bonxai/benchmark/README.md)
colcon build --packages-select rolling_bonxai --cmake-args -DBUILD_BENCHMARKS=ON

# run
ros2 launch rolling_bonxai_ros rolling_bonxai_mapping.launch.py   # composable node + Foxglove bridge
ros2 run rolling_bonxai_ros rolling_bonxai_server_node --ros-args --params-file rolling_bonxai_ros/params/rolling_bonxai_params.yaml
```

Compiled with `-Wall -Wextra -Wpedantic`. The node needs a `sensor_msgs/PointCloud2` source (`topic_in`) and TF from sensor frame to `frame_id`. `README.md` has the full parameter reference and policy-selection guidance.

## Package layout

- `bonxai_core` — vendored upstream header-only voxel grid (`Bonxai::VoxelGrid`, serialization). Treat as third-party.
- `bonxai_map` — `OccupancyMap`: log-odds (fixed-point ×1e6) probabilistic map over `VoxelGrid<CellOcc>`; `insertPointCloud` buffers hit endpoints then ray-traces free cells. Move-only.
- `bonxai_msgs`, `bonxai_ros` — upstream-style non-rolling ROS wrapper (plain Bonxai server); separate from the rolling variant.
- `rolling_bonxai` — the engine (no ROS dependency in the logic; ROS only for logging via `logger.hpp` interface).
- `rolling_bonxai_msgs` — `ChunkStats`, `QuickOccupancyStats`, `RollingState`.
- `rolling_bonxai_ros` — `RollingBonxaiServer` (rclcpp component + standalone `rolling_bonxai_server_node`), launch files, params; implements `ros_logger.hpp`.

## Rolling engine architecture (`rolling_bonxai`)

`RollingOccupancyMap` orchestrates everything. Each `insertPointCloud()` does:
1. **Occupancy update** — classify hit/miss, dispatch endpoints to the owning active chunk, ray-trace across chunk boundaries (chunk recomputed only on boundary crossing).
2. **Rolling update** — `pollPendingLoads()` promotes finished load futures; `TransitionManager` decides whether a chunk transition occurred; if so `performTransition()` asks the `ChunkLoadingPolicy` what to load/evict, issues async loads, and async-saves evicted *dirty* chunks (clean ones are just dropped).

Collaborators that must be understood together:
- `ChunkCoordinateSystem` — center-origin grid (chunk (0,0,0) centered on world origin); world ↔ voxel ↔ chunk conversion, neighbor enumeration.
- `TransitionManager` — 5-state hysteresis machine; transition fires only after penetrating `hysteresis_ratio × chunk_size` into the new chunk; jumps >1 chunk bypass hysteresis.
- `ChunkLoadingPolicy` (pure virtual, in `chunk_loading_policy_interface.hpp`, impls in `chunk_loading_policies.*`) — Neighborhood / PlanarNeighborhood / Temporal; answers `shouldLoad`/`shouldEvict` and load priority from a `PolicyContext`.
- `ManagedChunk` (`common.*`) — owns `unique_ptr<OccupancyMap>`; `getMutableMap()` marks dirty; `transferMapOwnership()` hands the grid out for saving without a copy.
- `ChunkStorage` (`chunk_storage.*`) — active-chunk container with Vector/HashMap/Deque implementations (compared by the benchmark).
- `AsyncChunkManager` — owns `FileStorageBackend` plus a priority-queue load pool and FIFO save pool; loads return `std::future`, polled non-blockingly from the main thread. Duplicate loads are rejected both by the caller (`active_chunks_`/`pending_loads_`) and the manager.
- `FileStorageBackend` — `<base_dir>/<x>_<y>_<z>/` containing a binary `.chunk` and YAML `.timestamp`; per-chunk files make concurrent access to different chunks safe.

The README's mermaid data-flow diagram traces point cloud → TF/PCL → engine → disk → published topics; note some file paths in its quick-reference table are stale (e.g. `rolling_bonxai_server.cpp` is under `rolling_bonxai_ros/src/`, `OccupancyMap` lives in `bonxai_map`).
