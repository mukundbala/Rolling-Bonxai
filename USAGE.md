# Using Rolling Bonxai

Building, launching, configuring and benchmarking Rolling Bonxai. See [README.md](README.md) for an overview and [ARCHITECTURE.md](ARCHITECTURE.md) for how it works.

## Contents

1. [Prerequisites](#prerequisites)
2. [Getting the Datasets](#getting-the-datasets)
3. [Launching Rolling Bonxai](#launching-rolling-bonxai)
4. [Running Experiments](#running-experiments)
5. [Parameter Reference](#parameter-reference)
6. [Policy Selection Quick Reference](#policy-selection-quick-reference)

---

## Prerequisites

Rolling Bonxai requires a **point cloud source** published as a `sensor_msgs/PointCloud2` topic in the ROS 2 graph. This is typically provided by a depth camera (e.g., ZED2i) or a LiDAR sensor. You must configure `topic_in` to match the topic your sensor is publishing on, and ensure TF transforms are available from your sensor frame to the `frame_id` (map frame).

**Required ROS 2 packages and dependencies:**

- `bonxai_core`, `bonxai_map`, `rolling_bonxai`, `rolling_bonxai_msgs`
- `rclcpp`, `sensor_msgs`, `tf2_ros`, `pcl_conversions`
- `yaml-cpp`

Build your workspace with:

```bash
colcon build --packages-select bonxai_core bonxai_map rolling_bonxai rolling_bonxai_msgs rolling_bonxai_ros
source install/setup.bash
```

**Devcontainer:** `.devcontainer/{humble,jazzy}` run the workspace in Docker, with the repo bind-mounted at `/ros2_ws/src/Rolling-Bonxai`. The container runs as the non-root user `ubuntu`, remapped to your host uid, so files created inside it belong to you on the host; use `sudo` for root-only commands. Containers built before this change ran as root. Their files are root-owned on the host; fix with `sudo chown -R "$(id -u):$(id -g)" <repo>`.

---

## Getting the Datasets

The experiments use five [TartanGround](https://tartanair.org/tartanground/) ground-robot trajectories, listed in `DEFAULT_TRAJECTORIES` in `datasets/scripts/download_tartanground.py`. One command downloads them, converts each one to a rosbag2/MCAP bag in `datasets/bags/`, and checks every bag has one scan per pose:

```bash
cd /ros2_ws
src/Rolling-Bonxai/datasets/scripts/fetch_datasets.sh                  # all five
src/Rolling-Bonxai/datasets/scripts/fetch_datasets.sh Hospital:P0005   # or specific ENV:TRAJ
```

| Trajectory | Path | Scans (10 Hz) | Download |
|---|---|---|---|
| Hospital:P0005 | 525 m | 3,444 | 4.2 GB |
| Downtown:P0005 | 784 m | 3,169 | 3.9 GB |
| ForestEnv:P0010 | 809 m | 3,940 | 6.0 GB |
| GreatMarsh:P0001 | 1,071 m | 3,989 | 5.2 GB |
| ModularNeighborhood:P0001 | 1,787 m | 6,782 | 8.2 GB |

- **Source:** the public Hugging Face dataset [`theairlabcmu/TartanGround`](https://huggingface.co/datasets/theairlabcmu/TartanGround), pinned to revision `388faf9c`, so every download gets the data the published results used. The data is CC-BY-4.0: credit TartanGround if you publish results based on it.
- **Disk:** about 27.5 GB downloaded (zips are deleted after unpacking), 26 GB unpacked in `datasets/data/` and 26 GB of bags. Both folders are git-ignored. `--no-images` skips the camera images, about half the download; the bags then have no `/camera` topic.
- **Dependencies:** ROS 2 with the MCAP storage plugin, and `huggingface_hub` (`pip install "huggingface_hub==1.33.0"`). The devcontainer image includes both. No TartanAir toolkit, torch or CUDA is needed.
- **Re-running is safe:** finished downloads and existing bags are skipped, and interrupted downloads resume. Delete a bag to rebuild it, e.g. after changing the converter.
- **Rate limits:** downloads work without an account. Setting `HF_TOKEN` gives higher Hugging Face rate limits.
- **Other trajectories:** `datasets/tartanground_omni_sizes.csv` lists all 440 omni trajectories and their sizes. Pass any `ENV:TRAJ` to the script, or add it to `DEFAULT_TRAJECTORIES`.

The steps can also be run on their own:
- `download_tartanground.py [ENV:TRAJ ...]`: download and unpack. Add `--dry-run` to show the plan and sizes, or `--modalities lidar metadata ...` for a subset.
- `tartanground_to_mcap.py datasets/data/<ENV>/Data_omni/<TRAJ>`: convert one trajectory. Its docstring documents the frame conversion (TartanGround's NED poses → ROS ENU `map`, FLU `base_link`).

**Bags converted before 2026-10-14 are wrong:** the converter read the NED poses as z-up, so the world came out vertically mirrored and slightly smeared (stairs going up appeared to go down). Delete and regenerate any such bags. The IMU data is not converted: its accelerations looked implausible (tens of m/s²), so check it before using it.

---

## Launching Rolling Bonxai

Every workflow below has a Rolling Bonxai launch file in `rolling_bonxai_ros` and an equivalent for plain (non-rolling) Bonxai in `bonxai_ros`, so the two can be compared on the same input. All of them start a Foxglove Bridge (`foxglove_port`, default `8765`) for visualization.

### 1. Checking a dataset bag

Dataset bags are TartanGround trajectories converted to rosbag2/MCAP with `datasets/scripts/tartanground_to_mcap.py`. They live in `datasets/bags/<Name>.bag` and contain `/lidar`, `/camera/image_raw/compressed`, `/tf` and `/tf_static`. To check a bag before mapping it, play it with the ground-truth trajectory and no mapper:

```bash
ros2 launch rolling_bonxai_ros bag_check.launch.py bag:=Hospital
ros2 launch rolling_bonxai_ros bag_check.launch.py bag:=ForestEnv rate:=4.0 loop:=true
```

The `map -> base_link` trajectory is published as a `nav_msgs/Path` on `/trajectory`. In Foxglove, check that the path is continuous and that `/lidar` stays aligned with the `map` frame as the path moves.

### 2. Mapping a dataset bag (with optional stats recording)

These launch files start the mapper on sim time, then play the bag with `--clock` after `play_delay` seconds:

```bash
ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital
ros2 launch bonxai_ros bonxai_bag_mapping.launch.py bag:=Hospital
```

Add `record:=true` to record the mapper's stats topics to an MCAP bag. Recording starts together with playback:

```bash
ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=Hospital record:=true
ros2 launch bonxai_ros bonxai_bag_mapping.launch.py bag:=Hospital record:=true record_voxels:=true
```

Recordings are named by the run's start time:

```
experiments/
├── rolling_bonxai/
│   ├── 2026-10-14_06-41-03/               # /rolling_bonxai/{quick_occupancy_stats, chunk_stats, transition_state}
│   └── chunks/2026-10-14_06-41-03/        # that run's saved chunks
└── bonxai/2026-10-14_06-41-30/            # /bonxai/occupancy_stats
```

Every Rolling Bonxai bag run, recorded or not, saves its chunks to a fresh `chunks/<datetime>/` folder, so no run starts from a previous run's map. Delete old chunk folders when you no longer need them.

- `record_voxels:=true` also records the occupied-voxel cloud. That cloud is the whole map on every message, so it makes the bags much larger.
- `experiments_dir` changes the output location.
- Recording continues after the bag finishes, until you press Ctrl-C.
- The recorder is also available on its own as `stats_recorder.launch.py` in either package, e.g. for live runs.

| Argument | Default | |
|---|---|---|
| `bag` | — | Bag name; `Hospital` → `<bags_dir>/Hospital.bag` |
| `bags_dir` | `datasets/bags` | |
| `rate` | `1.0` | Playback rate |
| `play_delay` | `3.0` | Seconds to wait for the mapper before playback |
| `record` / `record_voxels` | `false` | Stats recording, see above |
| `experiments_dir` | `experiments` | |

### 3. Mapping from a live camera (ZED2i example)

`rolling_bonxai_mapping.launch.py` starts the mapper in an `rclcpp_components` container, plus the Foxglove Bridge. Because the mapper runs as a component, it can use **zero-copy intra-process communication** with other components such as the ZED wrapper. The point cloud source is launched separately. For a ZED2i:

```bash
# terminal 1: camera (wraps zed_wrapper's zed_camera.launch.py with params/zed2i.yaml)
ros2 launch rolling_bonxai_ros zed2i_wrapper.launch.py

# terminal 2: mapper (bonxai_ros bonxai_mapping.launch.py for plain Bonxai)
ros2 launch rolling_bonxai_ros rolling_bonxai_mapping.launch.py

# optional, terminal 3: stats recording
ros2 launch rolling_bonxai_ros stats_recorder.launch.py
```

The mapper reads `rolling_bonxai_ros/params/rolling_bonxai_params.yaml`, which defaults to the ZED topics: `topic_in: /zed/zed_node/point_cloud/cloud_registered`, `base_frame_id: zed_camera_link`, `frame_id: map`. For any other sensor:
- set `topic_in` to its `sensor_msgs/PointCloud2` topic and `base_frame_id` to its body frame;
- make sure TF is available from the sensor frame to `frame_id`.

### Without a launch file

For debugging, the standalone executable takes the same parameters:

```bash
ros2 run rolling_bonxai_ros rolling_bonxai_server_node \
  --ros-args --params-file rolling_bonxai_ros/params/rolling_bonxai_params.yaml \
  -p topic_in:=/your/pointcloud/topic
```

---

## Running Experiments

`datasets/scripts/run_pipeline.sh` runs every dataset bag through both mappers, then extracts the stats to CSVs, plots them and checks that each run processed all of its bag's scans:

```bash
cd /ros2_ws
nohup src/Rolling-Bonxai/datasets/scripts/run_pipeline.sh > ~/batch.log 2>&1 &
tail -f ~/batch.log
```

- Defaults: all five environments, both mappers, half-speed playback (`--rate 0.5`) and a `perf` profile of every run (`--perf`). Extra arguments go to `run_experiments.py`, e.g. `--envs Hospital --mappers rolling_bonxai`.
- Runs use their own `ROS_DOMAIN_ID` (42) and Foxglove port (8766), so they don't mix with interactive sessions.
- `--perf` needs a build with frame pointers. The published results used Release with frame pointers and debug info (`-g` only adds debug sections; the machine code is unchanged, and it enables per-source-line `perf annotate`):
  `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer -g" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
  CMake caches the flags; rebuild with `-DCMAKE_CXX_FLAGS=""` to drop them. Expect about 0.1 GB of `.data` and 2 GB of `.perf` text per batch. The largest `.perf` file (Rolling Bonxai on ModularNeighborhood, ~1.4 GB) is too big for speedscope; use `perf report` on its `.data`.
- A run is valid only if it processed every scan in its bag: check the `complete` column of `summary.csv`. The script exits non-zero if any run is incomplete.

| Output | Contents |
|---|---|
| `experiments/<mapper>/<run>/` | Stats bag (MCAP, 1 Hz) |
| `experiments/rolling_bonxai/chunks/<run>/` | Chunks saved during the run |
| `experiments/profiles/<run>_<mapper>_<env>.{data,perf}` | `perf` profile and its text export (speedscope / Firefox Profiler) |
| `experiments/batches/<batch>.csv`, `experiments/logs/` | The batch's run list; launch logs |
| `experiments/results/<batch>/` | Per-run CSVs, `summary.csv`, one PNG per environment, `summary_memory.png` |

The steps can also be run on their own: `run_experiments.py`, then `extract_stats.py experiments/batches/<batch>.csv`, then `plot_stats.py experiments/results/<batch>`.

### Experiment configuration

The parameter files (`rolling_bonxai_ros/params/rolling_bonxai_params.yaml`, `bonxai_ros/params/bonxai_params.yaml`) hold the experiment values, and the bag launch files add the TartanGround overrides. The batch reads the same files as interactive launches: if you change them for a demo, change them back before running experiments.

| Setting | Value | Why |
|---|---|---|
| `occupancy.resolution` | 0.1 m (both mappers) | Practical for 200–300 m outdoor scenes |
| `sensor_model.max_range` | 9 m (both) | Equals Rolling Bonxai's guaranteed reach, see below |
| `chunk_size` / `radius` / `hysteresis_ratio` | 5 m / 2 / 0.2 | `(2 − 0.2) × 5 = 9 m = max_range`: no endpoint ever lands outside the loaded chunks |
| z filters | ±100 m (off) | Hospital and ForestEnv change height by 11–14 m |
| `stats.publish_occupied_voxels` | false (both) | Building the voxel cloud runs on the mapping thread and would distort CPU profiles |
| Bonxai `server.cleanup_interval_sec` | `1000000000.0` (off) | Rolling Bonxai has no periodic cleanup. Write it out in full: ROS reads `1.0e9` as a string and the node fails to load. |
| `qos_reliable` | true (set by the bag launches) | Best effort lost ~20% of the ~600 KB scans. Reliable uses a 50-scan queue. |
| Bag playback | `--rate 0.5`, `--delay 1.0`, `--wait-for-all-acked 5000` | At 1x Rolling Bonxai falls behind outdoors. Without `--delay` the first scan is lost; without `--wait-for-all-acked` the last one is. |
| Chunk folder | fresh per run: `experiments/rolling_bonxai/chunks/<run_name>` | A run never loads an earlier run's chunks |

### Known issues when running experiments

- **Startup TF race (Bonxai):** occasionally the first ~14 scans arrive before the bag's `/tf_static`, so the TF lookup fails ("`lidar` does not exist") and they are skipped. The fix would be for the bag launch to publish the identity `base_link → lidar` transform itself before playback.
- **Rolling Bonxai can fall behind even at half speed:** in dense stretches it can need more than 200 ms per scan. If it falls more than 50 scans behind, the oldest are dropped; play slower (`--rate 0.25`) if that matters.
- **Rolling Bonxai's first ~15 s** are spent catching up after startup, while it creates its initial chunks. Don't read throughput from that window.
- **Hit/miss counts aren't comparable** between the mappers: Rolling Bonxai counts every ray voxel as a miss, Bonxai only ray endpoints. The plots leave them out.

### Possible follow-ups

- **Planar policy, ±1 layer** (`use_relative`, `z_min: 1`, `z_max: 1`; 75 chunks instead of 125): neighbourhood radius 2 loads 50 chunks the sensor never reaches (see [Policy Selection](#policy-selection-quick-reference)).
- **More metrics:**
  - per-scan processing time (needs code in both mappers)
  - process RSS sampled by the runner
  - chunk loads, saves and on-disk size (the counters exist in `AsyncChunkManager` but aren't published)
  - chunk transitions marked on the memory plots
- **Performance fixes** identified by profiling: see [devlogs/2026-10-10/](devlogs/2026-10-10/).

---

## Parameter Reference

All parameters are scoped under `rolling_bonxai_server_node/ros__parameters/`. The YAML hierarchy below mirrors the parameter file structure.

### Core Frame & Topic

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `frame_id` | string | `"map"` | The global fixed frame. All map data is stored in this frame. |
| `base_frame_id` | string | `"base_footprint"` | The robot base frame used for TF lookups. |
| `topic_in` | string | `"/points"` | The `sensor_msgs/PointCloud2` topic to subscribe to. **Must match your sensor's output topic.** |
| `qos_reliable` | bool | `false` | **Unitless boolean.** `false` subscribes with `SensorDataQoS` (best effort), the right choice for live sensors. `true` subscribes reliably (queue of 50): use it for bag playback, where best effort loses large point clouds. A reliable subscriber cannot connect to a best-effort publisher. The bag launch files set it to `true`. |

### Occupancy Settings (`occupancy.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `occupancy.resolution` | double | `0.05` | **Meters (m).** Side length of each cubic voxel. Smaller = finer detail but higher memory usage. Typical range: 0.02–0.2 m. |
| `occupancy.occupancy_min_z` | double | `-100.0` | **Meters (m).** Points below this Z coordinate (in the map frame) are discarded before insertion. Useful to cut out the ground or underground clutter. |
| `occupancy.occupancy_max_z` | double | `100.0` | **Meters (m).** Points above this Z coordinate are discarded. Useful to remove ceiling returns or high-altitude noise. |
| `occupancy.occupancy_threshold` | double | `0.50` | **Unitless probability [0, 1].** A voxel with log-odds corresponding to this probability is considered on the boundary between occupied and free. Voxels above this are occupied; below are free. The default of 0.5 means equal probability. |

### Sensor Model (`sensor_model.*`)

These parameters control the Bayesian occupancy update. All probability values are converted internally to log-odds with a fixed-point scale of 1×10⁶.

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `sensor_model.max_range` | double | `30.0` | **Meters (m).** Points beyond this range are clipped to the maximum range and treated as free-space measurements rather than hits. |
| `sensor_model.hit` | double | `0.7` | **Unitless probability [0, 1].** Probability that a voxel is occupied given a sensor return. Higher values make the map respond faster to obstacles but may cause noise. |
| `sensor_model.miss` | double | `0.4` | **Unitless probability [0, 1].** Probability of occupancy given a ray passes through without a return. Lower values clear free space more aggressively. |
| `sensor_model.min` | double | `0.12` | **Unitless probability [0, 1].** Minimum clamping probability. Prevents a voxel from ever being considered more free than this. Stops the map from being over-confident about free space. |
| `sensor_model.max` | double | `0.97` | **Unitless probability [0, 1].** Maximum clamping probability. Prevents a voxel from becoming more occupied than this. Allows obstacles to eventually be cleared if the sensor reports free space there. |

### Rolling / Coordinate System (`rolling.coordinate_system.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `rolling.coordinate_system.chunk_size` | double | `5.0` | **Meters (m).** The side length of each cubic chunk. Each chunk wraps one `OccupancyMap`. Larger chunks reduce I/O frequency but increase per-chunk memory. Typical range: 2–20 m. |

### Rolling / Transition Manager (`rolling.transition_manager.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `rolling.transition_manager.hysteresis_ratio` | double | `0.2` | **Unitless [0.01, 1.0].** Fraction of `chunk_size` that the robot must penetrate into a new chunk before a chunk transition is confirmed. For example, with `chunk_size=5.0` and `hysteresis_ratio=0.2`, the robot must travel 1.0 m into the new chunk before any load/evict cycle fires. Higher values reduce I/O thrashing near boundaries at the cost of slightly delayed chunk loading. |

### Rolling / Loading Policy (`rolling.loading_policy.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `rolling.loading_policy.name` | string | `"neighbourhood"` | **Unitless string.** Selects the chunk management strategy: `"neighbourhood"` (or `"neighborhood"`), `"planar"`, `"temporal"`. Any other value logs a warning and falls back to neighbourhood. |

**Neighbourhood Policy** (`rolling.loading_policy.neighborhood.*`):

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `neighborhood.radius` | int | `2` | **Unitless (chunks).** Chebyshev radius — loads all chunks within this many chunk-lengths in every direction. Radius 1 → 26 neighbours; radius 2 → 124 neighbours. Increase for faster robots. **Used by all three policies**; there is no separate planar or temporal radius. |

**Planar Policy** (`rolling.loading_policy.planar.*`):

Loads chunks within `neighborhood.radius` horizontally and within the `z_min`/`z_max` layers vertically. `radius` also caps the vertical range (the check is a 3D Chebyshev distance), so the layers actually loaded are at most `min(z_min, radius)` below and `min(z_max, radius)` above the robot: planar can only trim the neighbourhood vertically, never extend it.

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `planar.z_min` | int | `1` | **Unitless (chunks).** Under `use_relative=false`: minimum absolute Z chunk coordinate to keep loaded. Under `use_relative=true`: number of chunk layers *below* the robot to keep loaded (must be ≥ 0). |
| `planar.z_max` | int | `3` | **Unitless (chunks).** Under `use_relative=false`: maximum absolute Z chunk coordinate. Under `use_relative=true`: number of chunk layers *above* the robot to keep loaded (must be ≥ 0). |
| `planar.use_relative` | bool | `false` | **Unitless boolean.** If `true`, `z_min` and `z_max` are relative offsets from the robot's current Z chunk rather than absolute chunk coordinates. Recommended for ground robots that operate at varying elevations. |

**Temporal Policy** (`rolling.loading_policy.temporal.*`):

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `temporal.age_weightage` | double | `0.3` | **Unitless [0.0, 1.0].** Blend factor between distance-based priority (0.0) and age-based priority (1.0). A value of 0.3 means 70% distance priority + 30% freshness priority when ordering load requests. |
| `temporal.max_age_s` | int | `300` | **Seconds (s).** Chunks older than this are considered stale and excluded from loading. Useful in dynamic environments. |

### Rolling / Storage (`rolling.storage.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `rolling.storage.full_save_folder_path` | string | `""` | **Unitless path string.** Full filesystem path where chunk files (`.chunk`) and timestamp files (`.timestamp`) are saved to and loaded from. Must be readable and writable by the process. Example: `"/home/robot/bonxai_chunks"`. |

### Rolling / Async I/O (`rolling.asyncio.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `rolling.asyncio.enable_io` | bool | `true` | **Unitless boolean.** Master switch for disk I/O. When `false`, evicted chunks are discarded and nothing is loaded from disk, so revisited areas start empty. Keep `true` for persistent, large-area mapping. |
| `rolling.asyncio.num_load_threads` | int | `3` | **Unitless (thread count).** Number of dedicated worker threads for loading chunks from disk. Higher values allow more concurrent loads but consume more CPU. |
| `rolling.asyncio.num_save_threads` | int | `1` | **Unitless (thread count).** Number of dedicated worker threads for saving chunks to disk. Saves are naturally serialised per-chunk (different files), so 1 is usually sufficient. |

### Server (`server.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `server.cleanup_interval_sec` | double | `300.0` | **Seconds (s).** How often the `/rolling_bonxai/clean_memory` maintenance cycle should be offered. Note: cleanup is actually triggered via the `/rolling_bonxai/clean_memory` service call; this setting is retained for future automatic scheduling. |

### Stats / Visualization (`stats.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `stats.enable_stats` | bool | `true` | **Unitless boolean.** Enables statistics publishers and the occupied-voxel point cloud publisher. Disable to reduce CPU overhead in production. |
| `stats.quick_stats` | bool | `true` | **Unitless boolean.** Read but currently ignored by Rolling Bonxai, which always publishes the quick statistics (`QuickOccupancyStats`: cell counts, memory, insertions). The plain Bonxai server (`bonxai_ros`) honours it: `false` adds full per-cell statistics, which are expensive on large maps. |
| `stats.publish_occupied_voxels` | bool | `true` | **Unitless boolean.** Publishes a `sensor_msgs/PointCloud2` of all currently occupied voxels on `/rolling_bonxai/occupied_voxels`. Disable if bandwidth is a concern. |
| `stats.publish_rate` | double | `1.0` | **Hertz (Hz).** Rate at which statistics and the occupied-voxel cloud are published. |

### Visualization (`visualization.*`)

| Parameter | Type | Default | Unit / Significance |
|-----------|------|---------|---------------------|
| `visualization.chunk_alpha` | double | `0.15` | **Unitless opacity [0, 1].** Opacity of the chunk boxes on `/rolling_bonxai/chunk_markers` (green dirty, blue clean, grey loading). Lower is more transparent. |

---

## Policy Selection Quick Reference

| Robot Type | Recommended Policy | Suggested Parameters |
|------------|--------------------|----------------------|
| Aerial drone (any environment) | `neighbourhood` | `radius=1` or `2` |
| Ground robot (flat indoor) | `planar` | `radius=1, z_min=0, z_max=1, use_relative=true` |
| Ground robot (variable elevation) | `planar` | `radius=2, z_min=1, z_max=1, use_relative=true` |
| Fast robot (>3 m/s) | `neighbourhood` | `radius=2` or `3` |
| Dynamic outdoor environment | `temporal` | `radius=2, max_age_s=300, age_weightage=0.3` |

**Radius sizing formula:**

```
radius = ceil(max_velocity_m_s × reaction_time_s / chunk_size_m)
```

For example: 4 m/s robot, 1.5 s reaction, 5 m chunks → `ceil(4 × 1.5 / 5) = ceil(1.2) = 2`.

**Sizing the vertical layers (planar):**

- Keep `z_min` and `z_max` ≤ `radius`; larger values have no effect (see the planar policy above).
- A sensor sees `max_range × sin(vertical FOV / 2)` above and below itself. One layer each way covers that whenever it is smaller than `chunk_size`. For example, a ±15° LiDAR at 9 m range sees ±2.3 m, so one layer each way covers it with 5 m chunks.
- For reach to stay ahead of the robot, keep `(radius − hysteresis_ratio) × chunk_size ≥ max_range`. Endpoints that land outside the loaded chunks are dropped, together with their whole ray.
- On the TartanGround ground-robot runs, neighbourhood with radius 2 kept 125 chunks loaded, but scans never reached the top and bottom layers (50 of the 125 chunks; checked on every 10th scan of all five routes). Planar with `z_min=1, z_max=1` would keep 75 chunks with the same coverage.
