# Bottlenecks: Bonxai (2026-10-10)

CPU profile of the plain (non-rolling) `BonxaiServer` on all five TartanGround bags. The profiles show that Bonxai
spends most of its CPU on essential free-space ray tracing. The largest avoidable cost is the 1 Hz stats publishing,
which walks the whole map every second.

## Setup
Same batch and setup as [bottleneck_rolling_bonxai.md](bottleneck_rolling_bonxai.md):
- **Batch:** `experiments/results/2026-10-14_14-58-05`, run on the frame-corrected bags.
  - ForestEnv and ModularNeighborhood each lost their first 14 scans to a startup race: the scans arrived before
    `/tf_static`. The profiles are unaffected.
  - An earlier batch on the mirrored bags gave the same ranking.
- **Profiling:** `perf record -F 99 --call-graph fp` for the whole run.
- **Build:** Release `-O3 -g -fno-omit-frame-pointer`.
- **Map parameters:** 0.1 m resolution, 9 m `max_range`, periodic cleanup off, stats published at 1 Hz with quick stats.
- **Method:** percentages are each function's own CPU time, summed over all five runs (40.5k samples).

CPU per scan: 8.2 ms (Hospital), 19.6 ms (Downtown), 23.8 ms (ForestEnv), 14.2 ms (GreatMarsh), 24.9 ms
(ModularNeighborhood). That is 5.2–7.1× less than Rolling Bonxai, and inside the 10 Hz budget everywhere.

**CPU by library:**

| Library | Share |
|---|---|
| `libbonxai_map` | 81% |
| kernel | 7.1% |
| `libgcc` (popcount) | 5.9% |
| server library | 2.8% |
| Fast DDS | 1.4% |

## Top 10

| # | Bottleneck | CPU | Where |
|---|---|---|---|
| 1 | Free-space ray tracing: per-voxel cell access in `updateFreeCells` (essential) | ~57% | `occupancy_map.cpp:373-401` |
| 2 | Stats timer walks the whole map every second | ~14% overall, 21% on ModularNeighborhood | `getQuickStats` → `activeCellsCount`, `memUsage` |
| 3 | Leaf lookup on cache miss (`getLeafGrid`: root hash map + inner grid) | ~12% | `bonxai.hpp` `Accessor::getLeafGrid` |
| 4 | Software popcount (`__popcountdi2` from libgcc) | ~5.9% (within #2) | `mask.hpp:187`, `:314-315` |
| 5 | Kernel time, mostly `perf`'s own sampling overhead | ~7% | `perf_event_update_userpage`, RCU |
| 6 | Endpoint updates (`addHitPoint` / `addMissPoint`, incl. position→coordinate conversion) | ~4.2% | `occupancy_map.cpp:335-371` |
| 7 | Per-point quaternion rotation and the server callback | ~2.1% | `bonxai_server.cpp` `pointcloud_callback` |
| 8 | Missed endpoints not de-duplicated: one ray per clipped point | rays ×(see below) | `occupancy_map.cpp:362-370`, `:395-398` |
| 9 | DDS receive of ~600 KB scans (Fast DDS + mutexes) | ~1.5% | `libfastrtps` |
| 10 | Cross-library `getLeafGrid` call through the PLT; PCL conversion | ~1.2% | `getLeafGrid@plt`, `pcl::fromPCLPointCloud2` |

### Details
1. **Free-space ray tracing:** a ray from the sensor to every unique hit voxel and every clipped miss, updating each
   voxel along the way (`RayIterator` with the `clearPoint` lambda; at `-O3` all of it is inlined into
   `updateFreeCells`). Within this ~57%, from `perf annotate -l`:
   - **~17% of total: leaf-cache check per voxel.** `getInnerKey`, then the `CoordT` comparison against the cached
     leaf (`grid_coord.hpp:178`, `:186`, `bonxai.hpp:427-428`, `:476-483`).
   - **~10%: leaf index and mask test** (`bonxai.hpp:443-445`, `:485`, `mask.hpp:330`).
   - **~10%: Bresenham stepping** (`occupancy_map.hpp:133-146`).
   - **~5%: log-odds update** (`occupancy_map.cpp:378-380`, `std::max`).

   This is essential work. Every ray starts at the sensor, so voxels near the sensor are revisited thousands of times
   per scan; `update_id` prevents double updates, but the traversal still costs. Cutting that would need a different
   ray-tracing scheme, not a local fix.
2. **Stats timer:** `stats_timer_callback` → `fill_stats_msg` → `getQuickStats()` calls `activeCellsCount()`, which
   popcounts every leaf mask in the map, plus `memUsage()`, which also walks every leaf.
   - The cost grows with map size and runs on the mapping thread (single-threaded executor), so it competes with scan
     processing.
   - It is about 21% of CPU on ModularNeighborhood (101 M voxels) by the end of the run.
   - **This is measurement overhead, not mapping.** It inflates Bonxai's CPU numbers relative to Rolling Bonxai, whose
     stats only walk the 125 loaded chunks.
   - Fix: keep active-cell and memory counters up to date as cells change, or compute them less often, or off the
     mapping thread.
3. **`getLeafGrid`:** when a ray enters a new 8³ leaf, the accessor does a root `unordered_map<CoordT>` lookup and an
   inner-grid lookup. Essential for this data structure.
4. **Software popcount:** `Mask::countOn` uses `__builtin_popcountll`. Without `-mpopcnt` (or `-march=x86-64-v2` or
   newer), GCC compiles that to a call into libgcc's software `__popcountdi2`. Fix: add `-mpopcnt`. That makes #2
   several times cheaper, and affects Rolling Bonxai's stats too.
5. **Kernel time (`perf` overhead):** mostly `perf_event_update_userpage` and timer/RCU work, i.e. the sampling
   itself. It shows up more for Bonxai because Bonxai does less work per scan. Exclude it when comparing.
6. **Endpoint updates:** `addHitPoint` / `addMissPoint`, including their `Vector3D` overloads (`posToCoord`).
   Essential.
7. **Per-point transform:** `pointcloud_callback` rotates each point with `Eigen::Quaternion * Vector3d`, which
   rebuilds the rotation for every point, then filters by z; the cloud also goes through `pcl::fromROSMsg`. Fix:
   convert the quaternion to a 3×3 matrix once per scan, and read the `PointCloud2` buffer directly.
8. **Missed endpoints aren't de-duplicated:** `addHitPoint` adds a voxel to `hit_coords_` only once per scan, but
   `addMissPoint` appends every clipped endpoint unconditionally, and `updateFreeCells` traces a ray to each one.
   Outdoors, 30–62% of points are beyond 9 m, and those are the longest rays (90 voxels each). With hits
   de-duplicated, Bonxai still traces 0.17–0.74 rays per point (Hospital to ModularNeighborhood), mostly from these
   misses. De-duplicating them like hits would cut the outdoor ray count noticeably. The change is small and doesn't
   change the map: duplicate rays revisit the same voxels, and `update_id` already blocks repeated updates.
9. **DDS:** receiving ~600 KB `PointCloud2` messages over reliable QoS. Transport cost; leave it.
10. **Minor:** the `getLeafGrid` call goes through the PLT because the template is instantiated in `libbonxai_map`;
    `pcl::fromPCLPointCloud2` is ~0.5%.

## Takeaways
- **Ray tracing dominates:** Bonxai's CPU is ~70% essential ray-tracing work (#1, #3, #6).
- **Avoidable costs:**
  - stats (#2, #4): ~14–21%, and growing with map size
  - the per-point transform (#7): ~2%
  - duplicate miss rays (#8): an algorithmic saving
- **For the Rolling Bonxai comparison:**
  - Bonxai's CPU numbers include the stats overhead (#2). With that removed, the Rolling ÷ Bonxai CPU ratio would be
    somewhat higher than the measured 5.2–7.1×.
  - Rolling Bonxai's extra cost is chunk bookkeeping and the lack of endpoint de-duplication; see its devlog.

## Reproduce
```bash
perf report -i experiments/profiles/<run>_bonxai_<env>.data --no-children --sort dso,sym --stdio -g none
perf report -i <same file> --children --sort sym --stdio -g none | grep -E "stats_timer|getQuickStats|updateFreeCells"
```
