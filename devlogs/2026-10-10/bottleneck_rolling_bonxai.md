# Bottlenecks: Rolling Bonxai (2026-10-10)

CPU profile of `RollingBonxaiServer` on all five TartanGround bags. The profiles show that most of Rolling Bonxai's
CPU goes to chunk bookkeeping repeated for every ray voxel, not to mapping.

## Setup
- **Batch:** `experiments/results/2026-10-14_14-58-05`, run on the frame-corrected bags.
  - Three runs fell ≤14 scans short of their bag (see the batch summary). Their profiles are still representative.
  - An earlier batch on the mirrored bags gave the same ranking.
- **Profiling:** `perf record -F 99 --call-graph fp`, attached to the mapper process for the whole run.
- **Build:** Release (`-O3 -DNDEBUG`) with `-fno-omit-frame-pointer -g`. `-g` only adds debug info; the machine code is
  unchanged.
- **Map parameters:** resolution 0.1 m, `max_range` 9 m, 5 m chunks, neighbourhood radius 2 (125 chunks), hysteresis 0.2.
- **Playback:** bags at 0.5x speed, reliable QoS.
- **Method:**
  - Percentages are each function's own CPU time, summed over all five runs (238k samples).
  - For code inlined into `updateOccupancy`, the split comes from `perf annotate -l`'s per-source-line breakdown.
  - The frame-pointer call graphs were not used: they misattribute callers in short functions that skip frame setup.
- **CPU time** is approximated as samples ÷ 99 Hz. Use it to compare runs, not as an absolute measurement.

## CPU per scan

| Env | Rolling Bonxai | Bonxai | Ratio |
|---|---|---|---|
| Hospital | 53 ms | 8.2 ms | 6.5× |
| Downtown | 139 ms | 19.6 ms | 7.1× |
| ForestEnv | 124 ms | 23.8 ms | 5.2× |
| GreatMarsh | 81 ms | 14.2 ms | 5.7× |
| ModularNeighborhood | 144 ms | 24.9 ms | 5.8× |

- **Over budget outdoors:** Downtown, ForestEnv and ModularNeighborhood average more than the 100 ms per scan that
  10 Hz allows.
- **Too slow even at half speed in dense stretches:** in one stretch of ModularNeighborhood (sim time ~100–160 s),
  Rolling Bonxai needed more than 200 ms per scan, the budget at 0.5x playback. It fell 54 scans behind, more than its
  50-scan queue, and 4 scans were dropped.
- **Everything is on the mapping thread:** the kernel is 1.6% of samples; disk I/O and DDS are negligible.
  Chunk loading and saving is not a cost.
- **Profiles are consistent:** they agree across environments within a few percent.

## Top 10

Almost everything is in the per-voxel lambda in `updateOccupancy`
([rolling_occupancy_map.cpp:204-233](../../rolling_bonxai/src/rolling_occupancy_map.cpp)), which runs once for every
voxel along every ray.

| # | Bottleneck | CPU | Where |
|---|---|---|---|
| 1 | `touched_chunks.insert()` for every ray voxel | ~29% | `rolling_occupancy_map.cpp:226` |
| 2 | `active_chunks_.find()` for every ray voxel | ~11% | `:223`, `chunk_storage.cpp:143` |
| 3 | One ray per point, with no de-duplication of endpoints | ×1.4–6 rays | `rolling_occupancy_map.cpp:144-233` |
| 4 | Per-voxel calls into other files through the shared library's PLT (`isMapValid`, `getMutableMap`, `markDirty`, `ensureAccessorValid`) | ~12% | `:224-225`, `common.cpp:120-157` |
| 5 | `ChunkCoord::operator==` cannot be inlined | ~11% | `common.cpp:17` |
| 6 | Atomic `markDirty()` store with full ordering, every voxel | ~1.6% (within #4) | `common.cpp:141` |
| 7 | Grid update in `addMissPoint` (essential) | ~19% | `occupancy_map.cpp:355-371`, `getLeafGrid` |
| 8 | Bresenham stepping (essential) | ~3% | `occupancy_map.hpp:133-149` |
| 9 | Chunk boundary check every voxel | ~3% | `rolling_occupancy_map.cpp:209-211` |
| 10 | `miss_coords_.push_back()` for every ray voxel, a list that is never read | ~1–2% CPU, 68–203 MiB of memory | `occupancy_map.cpp:370` |

### Details
1. **`touched_chunks.insert()`:** an `unordered_set<ChunkCoord>` insert on every voxel. At `-O3`, GCC moves the insert,
   with its hashing inlined, into a separate clone (`_Hashtable::_M_insert_unique [clone .isra.0]`), which is the single
   hottest symbol. The chunk is almost always already in the set, because consecutive voxels lie in the same 5 m chunk.
   Inserting only when the ray enters a new chunk removes nearly all of this cost.
2. **`active_chunks_.find()`:** hashes the chunk coordinate (FNV-1a) and searches its bucket on every voxel. The
   result is the same as for the previous voxel unless the boundary check (#9) just fired. Fix: look the chunk up once
   per chunk crossing and cache the `ManagedChunk*` together with the voxel bounds.
3. **One ray per point:** Rolling Bonxai traces a ray for every point. Bonxai adds each hit voxel only once per scan
   (`addHitPoint` checks `update_id`) and then traces rays to those unique voxels. Measured on every 50th scan in the
   corrected frames (rechecked after the frame fix; the values didn't change), with points clipped at 9 m:

   | Env | Unique hit voxels ÷ hit points | Rays Bonxai traces ÷ rays Rolling Bonxai traces |
   |---|---|---|
   | Hospital | 0.09 | 0.17 |
   | Downtown | 0.11 | 0.60 |
   | ForestEnv | 0.27 | 0.73 |
   | GreatMarsh | 0.17 | 0.41 |
   | ModularNeighborhood | 0.21 | 0.74 |

   Every other item in this list scales with the number of rays traced. This explains much of Hospital's high ratio:
   scans are short-range indoors, so many points land in the same voxels. Fix: collect unique endpoint voxels first,
   as Bonxai does, then trace one ray per unique voxel.
4. **Per-voxel cross-library calls:** `isMapValid()`, `getMutableMap()` (which calls `markDirty()`) and
   `ensureAccessorValid()` are out-of-line functions in other files, each called through the shared library's PLT on
   every voxel. Lines 224-225 alone account for about 31% of `updateOccupancy`'s own time. Fix: check validity and
   mark the chunk dirty once per chunk crossing, not per voxel; move the trivial accessors into headers.
5. **`ChunkCoord::operator==`:** defined in `common.cpp`, so both hash tables make a real function call (through the
   PLT) for every key comparison. Fix: define it inline in `common.hpp`.
6. **`markDirty()`:** an `is_dirty_.store(true)` with sequential-consistency ordering, which is a locked instruction
   on x86, on every voxel. Fix: once per chunk per scan, with `memory_order_relaxed`.
7. **Grid update (`addMissPoint`):** the real mapping work: cell lookup, log-odds update, and `getLeafGrid` when a ray
   enters a new leaf. Bonxai does the same work. Items #3, #4 and #10 are the removable overhead around it.
8. **Bresenham stepping:** essential.
9. **Chunk boundary check:** six integer comparisons per voxel. Cheap, and it should stay: it is what tells the code
   when #1, #2 and #4 need to run.
10. **`miss_coords_.push_back()`:** `addMissPoint` appends every ray voxel to `miss_coords_`, and
    `updateFreeCellsPreRayTrace` (`occupancy_map.cpp:403-410`) only clears the list; Rolling Bonxai never reads it.
    The list keeps its peak capacity in each of the 125 loaded chunks. That makes up the 68–203 MiB of "ray-tracing
    buffers" at the end of each run, against 15–64 MiB of actual map (never above 66 MiB). Fix: give Rolling Bonxai a
    miss update that doesn't append to the list.

**Minor (under 2% combined):** per-point `positionToVoxelCoordinate` and `positionToChunkCoordinate`; the point
transform in `pointcloudCallback`.

## Expected impact (estimate, not measured)
- **Speed:** #1, #2, #4, #5, #6 and #9 are about 60–65% of CPU, and all can move to "once per chunk crossing" (#9 stays,
  but becomes the only per-voxel check). That should make Rolling Bonxai 2–2.5× faster. Adding #3 multiplies on top:
  up to about 6× fewer rays indoors, 1.4× on the open outdoor routes.
- **Memory:** #10 would cut total memory to near the map size.
- **Results:** none of these change the map. To confirm, check that saved chunks come out byte-identical before and
  after.

## Reproduce
```bash
perf report -i experiments/profiles/<run>_rolling_bonxai_<env>.data --no-children --sort sym --stdio -g none
perf annotate -i <same file> --stdio -l --no-source "<symbol>"   # per-source-line split of an inlined hot function
```
