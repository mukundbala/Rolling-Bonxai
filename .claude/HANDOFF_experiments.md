# Experiments handoff: Rolling Bonxai vs Bonxai on TartanGround (2026-10-14)

## State
- `experiments/` is reset to an empty skeleton: `bonxai/`, `rolling_bonxai/chunks/`, `batches/`, `logs/`, `results/`, `profiles/`. Each folder has a `.gitkeep`, which is tracked; everything else in them is git-ignored. `datasets/data/` and `datasets/bags/` work the same way.
- Next: the user runs the batch (command below). It has never completed. In the earlier attempt the user deleted files by hand; that was not a bug.
- Decided 2026-10-14: neighbourhood policy, and no further code fixes before the batch.

## Run
All commands run from `/ros2_ws` with `source install/setup.bash`.
```bash
# Screen capture (map visible in Foxglove at ws://localhost:8765; layout in rolling_bonxai_ros/foxglove/)
ros2 launch rolling_bonxai_ros rolling_bonxai_bag_mapping.launch.py bag:=ModularNeighborhood record:=true publish_voxels:=true rate:=0.5

# Batch: all 5 envs x {rolling_bonxai, bonxai}, perf attached to every run. ~2.5 h at 0.5x.
python3 src/Rolling-Bonxai/datasets/scripts/run_experiments.py --rate 0.5 --perf
python3 src/Rolling-Bonxai/datasets/scripts/extract_stats.py src/Rolling-Bonxai/experiments/batches/<batch>.csv
python3 src/Rolling-Bonxai/datasets/scripts/plot_stats.py src/Rolling-Bonxai/experiments/results/<batch>
```
- Batch runs use `ROS_DOMAIN_ID=42` and Foxglove port 8766, so they don't mix with interactive runs.
- Output layout:
  - `experiments/<mapper>/<run_name>/`: stats bag (mcap)
  - `experiments/rolling_bonxai/chunks/<run_name>/`: that run's chunks
  - `experiments/batches/<batch>.csv`: list of the batch's runs
  - `experiments/logs/`: launch logs
  - `experiments/profiles/<run_name>_<mapper>_<env>.{data,perf}`: perf profile (`.data`) and its text export for speedscope / Firefox Profiler (`.perf`)
  - `experiments/results/<batch>/`: per-run CSVs, `summary.csv`, one PNG per env
- Bag-launch arguments: `bag rate record record_voxels publish_voxels exit_on_end run_name experiments_dir`.
- A run is valid only if its final `point_clouds_processed` equals the bag's scan count. `summary.csv` has a `complete` column for this.

| Bag | Scans | Path (m) | Duration at 1x (s) |
|---|---|---|---|
| Hospital | 3444 | 525 | 344 |
| Downtown | 3169 | 784 | 317 |
| ForestEnv | 3940 | 809 | 394 |
| GreatMarsh | 3989 | 1071 | 399 |
| ModularNeighborhood | 6782 | 1787 | 678 |

## Agreed
- **Params**, set in `rolling_bonxai_ros/params/rolling_bonxai_params.yaml` and `bonxai_ros/params/bonxai_params.yaml`:
  - **Both mappers:**
    - resolution 0.1
    - max_range 9.0
    - z filters ±100 (off)
    - sensor model unchanged
    - `publish_occupied_voxels: false`
    - stats 1 Hz, quick
  - **Rolling only:**
    - chunk_size 5.0, neighbourhood radius 2, hysteresis 0.2. That makes (2 − 0.2) × 5 = 9 m = max_range, so endpoints never land outside the loaded chunks.
    - policy `neighbourhood`; both spellings are now accepted, and an unknown name logs a warning
    - `enable_io: true`
  - **Bonxai only:** `cleanup_interval_sec: 1e9`, effectively off, because Rolling has no periodic cleanup. Write it as `1000000000.0`: the ROS params parser reads `1.0e9` as a string and the node fails to load.
- **Topics and frames for TartanGround** (`/lidar`, `map`, `base_link`) are set by the bag launches. The YAML files keep the ZED defaults for live use.
- **`qos_reliable`:** a new param on both mappers, true in the bag launches. Best-effort QoS dropped about 20% of the 600 KB scans. Reliable uses a queue of 50 to absorb Rolling's slow start.
- **Bag play:** `--delay 1.0` (otherwise the first scan is lost) and `--wait-for-all-acked 5000` (otherwise the last scan is lost).
- **Chunk folder:** fresh per run, under `experiments/rolling_bonxai/chunks/<run_name>`.
- **Run experiments at rate 0.5.** At 1x, Rolling falls behind outdoors (about 7.5 of 10 scans/s on ModularNeighborhood, with voxel publishing on). Memory and cell counts don't depend on rate.
- **Code changes made:**
  - Bonxai respects `publish_occupied_voxels`.
  - The per-scan INFO log in `updateOccupancy` was removed.
  - `point_clouds_processed` was added to `QuickOccupancyStats` and to both Foxglove layouts.
- **Profiling:** every batch run is profiled with `perf record -F 99 --call-graph fp`.
  - The mapper packages (`bonxai_map rolling_bonxai rolling_bonxai_ros bonxai_ros`) are built with `-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer`. The flag is cached in CMake, so it stays until you rebuild with `-DCMAKE_CXX_FLAGS=""`. Both mappers get the same overhead (~1-2%).
  - Expect about 0.1 GB of `.data` and about 2 GB of `.perf` text for the whole batch. Rolling ModularNeighborhood's `.perf` alone is about 1.4 GB, which may be too big for speedscope; use `perf report` on its `.data` instead.
  - `perf` is `/usr/local/bin/perf`, a symlink to `/usr/lib/linux-tools/6.8.0-147-generic/perf`. The Ubuntu wrapper refuses to run on the Fedora host kernel. The symlink and the `linux-tools` install are not in the Dockerfile.
  - First profile (Downtown, 1x): about 47% of mapping time goes to the per-ray-voxel `touched_chunks.insert` and `active_chunks_.find` in `updateOccupancy`. Proposed fix: look up the chunk once per chunk crossing. Deferred: no code fixes before the batch.
- **Plots** are drawn against `point_clouds_processed`, with distance on the top axis: memory (total and grid), active cells, bytes per cell, Rolling chunk counts.
  - hit/miss counts are not compared: Rolling counts every ray voxel, Bonxai only endpoints.

## Not yet agreed
1. **Planar ±1 as a follow-up (optional).** Neighbourhood was chosen for the batch. The LiDAR's vertical FOV is ±15°, so rays reach only ±2.3 m at 9 m range. Neighbourhood's ±2 layers (50 of the 125 chunks) are never reached; ModularNeighborhood touches only layers 0 and −1. Report this as a finding; planar `use_relative`, z_min 1, z_max 1 (75 chunks) would be the follow-up run.
2. **Extra plots and metrics** (suggested):
   - per-scan processing time (needs code in both mappers; highest value)
   - process RSS sampled by the runner
   - chunk loads/saves and on-disk size (counters exist in `AsyncChunkManager` but aren't published)
   - transitions marked on the memory plot
   - cross-environment summary bars
3. **Save loaded chunks on shutdown.** Currently the up-to-125 loaded chunks are not written when a run ends.
4. **Planar policy bug.** `dz` is included in the radius check, so the vertical range is capped at `radius`. `getDescription` miscounts layers in relative mode. The README documents a `planar.radius` param that doesn't exist, and its "variable elevation" example (z_max 3) doesn't do what it says. Proposed fix: apply radius to x/y only.
5. **Rays to endpoints outside the loaded chunks are dropped entirely**, including the free space they would have marked inside. It's harmless while the window rule holds; changing it is an algorithm change.
6. **Rolling ignores `stats.quick_stats` and `server.cleanup_interval_sec`**, and has no verbose stats message.

## Gotchas
- **Memory finding so far:** Rolling uses about 10× Bonxai's memory on a short Hospital clip (113 vs 11 MiB), mostly per-chunk buffers (about 0.46 MiB each).
- **Rolling's first ~15 s** are spent catching up after startup; don't read throughput from that window.
- **matplotlib** was installed with apt and is not in the Dockerfile.
- **`rolling_bonxai_ros/launch/zed2i_wrapper.launch.py`** is an exact copy of the `bonxai_ros` one. It loads `zed2i.yaml` from `bonxai_ros`, and its usage text names a launch file that doesn't exist. Untested.

## Dataset notes (from the old datasets/HANDOFF.md)
- **Re-downloading:** run the downloader on the host, not in the container:
  `~/src/tartan-venv/bin/python -I datasets/scripts/download_tartanground.py ENV:TRAJ ...`
  It needs `~/src/tartanairpy`, cloned with `--recursive`, plus the uv venv `~/src/tartan-venv` (Python 3.9).
- **Sizes:** `datasets/tartanground_omni_sizes.csv` lists the sizes of all 440 omni trajectories (412 GB in total).
- **LiDAR frame:** the scans are mirrored relative to the poses. The converter fixes this; see its docstring.
- **IMU:** not converted. Its accelerations looked implausible (tens of m/s²), so check them before use.
